#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用 DAPLink 直接在 SWD 上"手敲" I2C —— 不经过固件，不需要重新烧录。

原理：把 CPU 停下来，GPIO 的状态会保持住；然后从 PC 端通过 SWD 反复写
GPIOB->BSRR 翻转 SCL/SDA、读 GPIOB->IDR 采电平，就能完整跑一次 I2C 事务。
I2C 没有最低时钟要求，所以哪怕每个 bit 花 1ms 也完全合法。

这样能拿到固件给不了的东西：
  * SCL/SDA 空闲时到底是高还是低（低了 = 被谁拉死了）
  * 地址字节有没有人应答（ACK 位是不是被从机拉低）
  * 任何一个寄存器读回来的真实字节

PY32F002B 地址（来自 py32f002bx5.h）：
  GPIOB_BASE = 0x50000400   MODER+0x00 OTYPER+0x04 IDR+0x10 ODR+0x14 BSRR+0x18
  PB3 = SCL, PB4 = SDA
"""

import sys
import time
import argparse
from contextlib import contextmanager

from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

GPIOB = 0x50000400
MODER = GPIOB + 0x00
OTYPER = GPIOB + 0x04
OSPEEDR = GPIOB + 0x08
PUPDR = GPIOB + 0x0C
IDR = GPIOB + 0x10
ODR = GPIOB + 0x14
BSRR = GPIOB + 0x18

SCL_BIT = 3
SDA_BIT = 4


@contextmanager
def preserve_bus(target):
    """诊断结束（含异常）恢复 GPIO 与原来的 CPU 运行状态。"""
    running = target.get_state() == Target.State.RUNNING
    target.halt()
    saved = {}
    try:
        for reg in (MODER, OTYPER, OSPEEDR, PUPDR, ODR):
            saved[reg] = target.read_memory(reg)
        yield
    finally:
        try:
            if len(saved) == 5:
                mask = (1 << SCL_BIT) | (1 << SDA_BIT)
                # 先恢复输出锁存，再恢复模式，避免恢复输出时出现低脉冲。
                target.write_memory(BSRR, (saved[ODR] & mask) |
                                    ((~saved[ODR] & mask) << 16))
                for reg in (OTYPER, OSPEEDR, PUPDR, MODER):
                    target.write_memory(reg, saved[reg])
                target.flush()
        finally:
            if running:
                target.resume()


class Bus:
    def __init__(self, target):
        self.t = target

    def w(self, addr, val):
        self.t.write_memory(addr, val & 0xFFFFFFFF)

    def r(self, addr):
        return self.t.read_memory(addr) & 0xFFFFFFFF

    # --- 引脚操作 ---
    def scl(self, lvl):
        self.w(BSRR, (1 << SCL_BIT) if lvl else (1 << (SCL_BIT + 16)))
        if lvl:
            deadline = time.monotonic() + 0.05
            while not self.rd_scl():
                if time.monotonic() >= deadline:
                    raise RuntimeError("SCL did not rise; scan stopped")

    def sda(self, lvl):
        self.w(BSRR, (1 << SDA_BIT) if lvl else (1 << (SDA_BIT + 16)))

    def rd_sda(self):
        return (self.r(IDR) >> SDA_BIT) & 1

    def rd_scl(self):
        return (self.r(IDR) >> SCL_BIT) & 1

    # --- I2C 基本动作 ---
    def setup(self):
        """PB3/PB4 配成开漏输出、输出高（释放总线）"""
        # 先选开漏并释放输出锁存，再切到输出，避免瞬间推挽驱动总线。
        self.w(OTYPER, self.r(OTYPER) | (1 << SCL_BIT) | (1 << SDA_BIT))
        self.scl(1)
        self.sda(1)
        for bit in (SCL_BIT, SDA_BIT):
            m = self.r(MODER)
            m &= ~(3 << (bit * 2))
            m |= (1 << (bit * 2))          # 01 = 通用输出
            self.w(MODER, m)

    def start(self):
        self.sda(1)
        self.scl(1)
        self.sda(0)
        self.scl(0)

    def stop(self):
        self.sda(0)
        self.scl(1)
        self.sda(1)

    def write_byte(self, b):
        """返回 True = 从机给了 ACK"""
        for i in range(7, -1, -1):
            self.sda((b >> i) & 1)
            self.scl(1)
            self.scl(0)
        self.sda(1)                 # 释放 SDA，等从机拉低
        self.scl(1)
        ack = (self.rd_sda() == 0)
        self.scl(0)
        return ack

    def read_byte(self, ack):
        val = 0
        self.sda(1)
        for _ in range(8):
            self.scl(1)
            val = (val << 1) | self.rd_sda()
            self.scl(0)
        self.sda(0 if ack else 1)
        self.scl(1)
        self.scl(0)
        self.sda(1)
        return val

    def recover(self):
        """9 个时钟把卡住的从机打醒"""
        self.sda(1)
        for _ in range(9):
            self.scl(1)
            self.scl(0)
        self.stop()

    # --- 事务 ---
    def probe(self, addr7):
        if not (self.rd_scl() and self.rd_sda()):
            raise RuntimeError("Bus not idle before address probe")
        self.start()
        ack = self.write_byte(addr7 << 1)
        self.stop()
        if not (self.rd_scl() and self.rd_sda()):
            raise RuntimeError("Bus stuck after STOP; ACK result discarded")
        return ack

    def read_reg(self, addr7, reg, length=1):
        self.start()
        if not self.write_byte(addr7 << 1):
            self.stop()
            return None, "地址 NAK"
        if not self.write_byte(reg):
            self.stop()
            return None, "寄存器 NAK"
        self.start()
        if not self.write_byte((addr7 << 1) | 1):
            self.stop()
            return None, "读地址 NAK"
        buf = bytearray()
        for i in range(length):
            buf.append(self.read_byte(i < length - 1))
        self.stop()
        return bytes(buf), None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frequency", type=int, default=50000, help="SWD clock Hz")
    ap.add_argument("--uid", default=None, help="Debug probe unique ID")
    args = ap.parse_args()
    t0 = time.time()
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.uid,
        target_override="cortex_m",
        options={"frequency": args.frequency, "connect_mode": "attach",
                 "resume_on_disconnect": False},
        blocking=False,
    )
    if session is None:
        print("连不上探针。检查 DAPLink 是否压住 SWD 触点、板子是否上电。")
        return 1

    with session, preserve_bus(session.target):
        target = session.target
        target.halt()
        bus = Bus(target)

        print("=" * 60)
        init_odr = bus.r(ODR)
        init_mod = bus.r(MODER)
        print("GPIOB 现场（CPU 已停）：")
        print("  MODER  = 0x%08X   PB3=%d PB4=%d (1=输出 2=复用)"
              % (init_mod, (init_mod >> 6) & 3, (init_mod >> 8) & 3))
        print("  OTYPER = 0x%08X   PB3=%d PB4=%d (1=开漏)"
              % (bus.r(OTYPER), (bus.r(OTYPER) >> SCL_BIT) & 1, (bus.r(OTYPER) >> SDA_BIT) & 1))
        print("  ODR    = 0x%08X   PB3=%d PB4=%d"
              % (init_odr, (init_odr >> SCL_BIT) & 1, (init_odr >> SDA_BIT) & 1))

        # 释放总线后看真实电平：这才是"上拉在不在、线有没有被拉死"的答案
        bus.setup()
        scl_lvl = bus.rd_scl()
        sda_lvl = bus.rd_sda()
        print("  空闲电平：SCL=%d SDA=%d   %s"
              % (scl_lvl, sda_lvl,
                 "两条都被上拉住了，总线是干净的" if (scl_lvl and sda_lvl)
                 else "有条线是低的 —— 被谁拉死了（从机没电/接反/短路）"))

        print("-" * 60)
        if not (scl_lvl and sda_lvl):
            bus.recover()
            print("  做了 9 个恢复时钟，之后：SCL=%d SDA=%d"
                  % (bus.rd_scl(), bus.rd_sda()))
            if not (bus.rd_scl() and bus.rd_sda()):
                print("总线仍未释放，停止扫描，避免把 SDA 卡低误判成从机 ACK。")
                return 2

        print("扫地址（谁应答谁存在）：")
        found = []
        for a in range(0x08, 0x78):
            ack = bus.probe(a)
            if ack:
                found.append(a)
            print("  0x%02X : %s" % (a, "应答 ACK" if ack else "无应答"))
        print("-" * 60)
        if not found:
            print("结论：0x08..0x77 的 112 个普通 7 位地址均无应答。")
            print("  → 传感器没供电 / SDA·SCL 接错脚 / 模块没插好，")
            print("    还需在模块侧测供电、nCS 及两根线，不能仅凭 MCU 端电平判定硬件正常。")
        else:
            for a in found:
                for reg in (0x00, 0x0F, 0x75):
                    v, err = bus.read_reg(a, reg)
                    print("  0x%02X reg[0x%02X] = %s"
                          % (a, reg, ("0x%02X" % v[0]) if v else ("失败：" + err)))
                print("  仅采集身份线索，不向未知器件写入 MPU 配置。")

        print("=" * 60)
        print("用时 %.2f 秒" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
