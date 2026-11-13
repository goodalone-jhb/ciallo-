#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全引脚普查 + 任意引脚对 I2C 扫描。

用途：当"总线上没人应答"时，先确认传感器到底接在哪两根线上 ——
有可能是焊的时候接到了别的脚，或者根本不是 PB3/PB4。

第一步：把所有 GPIO 配成输入 + 内部下拉，读 IDR。
        能顶住 40k 下拉还读 1 的，一定是"外面有上拉"或"被别处驱动"，这类脚才有嫌疑。
第二步：在嫌疑脚里两两组合当 SCL/SDA，去探 0x68/0x69。
        哪一对能探到应答，传感器就在那一对上。

安全边界：PA2 / PB6 是 SWD 引脚，一动就断线，全程不碰。
"""

import time

from pyocd.core.helpers import ConnectHelper

PORTS = {"PA": 0x50000000, "PB": 0x50000400}
MODER, OTYPER, PUPDR, IDR, ODR, BSRR = 0x00, 0x04, 0x0C, 0x10, 0x14, 0x18

SKIP = {("PA", 2), ("PB", 6)}          # SWD，绝不触碰


def pins():
    for name in PORTS:
        for bit in range(8):
            if (name, bit) not in SKIP:
                yield name, bit


def main():
    session = ConnectHelper.session_with_chosen_probe(
        target_override="cortex_m",
        options={"frequency": 1000000, "connect_mode": "halt"},
        blocking=False,
    )
    if session is None:
        print("连不上探针")
        return 1

    with session:
        t = session.target
        t.halt()

        def w(a, v):
            t.write_memory(a, v & 0xFFFFFFFF)

        def r(a):
            return t.read_memory(a) & 0xFFFFFFFF

        # ---------- 第一步：全脚下拉，看谁顶得住 ----------
        for name, base in PORTS.items():
            mod = r(base + MODER)
            pup = r(base + PUPDR)
            for bit in range(8):
                if (name, bit) in SKIP:
                    continue
                mod &= ~(3 << (bit * 2))          # 输入
                pup &= ~(3 << (bit * 2))
                pup |= (0b10 << (bit * 2))        # 内部下拉
            w(base + MODER, mod)
            w(base + PUPDR, pup)

        time.sleep(0.05)
        suspects = []
        print("=" * 58)
        print("第一步：全脚内部下拉后仍为高 = 外部有上拉（嫌疑脚）")
        for name, base in PORTS.items():
            idr = r(base + IDR)
            row = []
            for bit in range(8):
                if (name, bit) in SKIP:
                    row.append("--")
                    continue
                v = (idr >> bit) & 1
                row.append("H" if v else ".")
                if v:
                    suspects.append((name, bit))
            print("  %s0..7 : %s" % (name, " ".join(row)))
        print("  嫌疑脚：%s" % ", ".join("%s%d" % p for p in suspects))

        if len(suspects) < 2:
            print("\n不到两个疑似带上拉的脚 —— 传感器那两根线要么没接，要么不在 MCU 上。")
            t.resume()
            return 0

        # ---------- 第二步：两两组合去探 ----------
        print("-" * 58)
        print("第二步：把嫌疑脚两两当 SCL/SDA，探 0x68 / 0x69")

        def cfg_out(name, bit):
            base = PORTS[name]
            mod = r(base + MODER)
            mod &= ~(3 << (bit * 2))
            mod |= (1 << (bit * 2))               # 通用输出
            w(base + MODER, mod)
            w(base + OTYPER, r(base + OTYPER) | (1 << bit))   # 开漏
            w(base + BSRR, 1 << bit)              # 释放为高

        def setpin(name, bit, lvl):
            w(PORTS[name] + BSRR, (1 << bit) if lvl else (1 << (bit + 16)))

        def getpin(name, bit):
            return (r(PORTS[name] + IDR) >> bit) & 1

        def probe(scl, sda, addr7):
            def scl_set(l):
                setpin(scl[0], scl[1], l)

            def sda_set(l):
                setpin(sda[0], sda[1], l)

            sda_set(1); scl_set(1); sda_set(0); scl_set(0)      # START
            val = addr7 << 1
            for i in range(7, -1, -1):
                sda_set((val >> i) & 1)
                scl_set(1)
                scl_set(0)
            sda_set(1)
            scl_set(1)
            ack = (getpin(sda[0], sda[1]) == 0)                 # ACK = 从机把 SDA 拉低
            scl_set(0)
            sda_set(0); scl_set(1); sda_set(1)                  # STOP
            return ack

        hits = []
        for i, scl in enumerate(suspects):
            for sda in suspects:
                if sda == scl:
                    continue
                cfg_out(*scl)
                cfg_out(*sda)
                for addr in (0x68, 0x69):
                    if probe(scl, sda, addr):
                        hits.append((scl, sda, addr))
                        print("  SCL=%s%d  SDA=%s%d  ->  0x%02X 应答！"
                              % (scl[0], scl[1], sda[0], sda[1], addr))

        print("-" * 58)
        if hits:
            print("找到从机了：%s" % hits)
        else:
            print("所有引脚组合都没有应答 —— 传感器在这个 MCU 上看不到。")
            print("  剩下的可能：模块没供电 / 模块的 SDA·SCL 断线 / 模块坏了。")

        t.resume()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
