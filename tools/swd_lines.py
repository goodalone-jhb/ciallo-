#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
判断 PB3/PB4 上到底有没有"外部"上拉（4.7k），还是只有 MCU 的内部上拉。

做法：把引脚配成纯输入，分三种偏置各读一遍 IDR：
    无上下拉(00) → 读到 1 说明有外部上拉在拉；读到 0/乱跳说明线是浮的
    下拉(10)     → 外部 4.7k 会顶住不被 40k 下拉拉低，仍是 1
    上拉(01)     → 一定读到 1（对照组）

这一步能分开"模块的 4.7k 在模块板上、而模块没通电"和"主板上另有一只上拉"。
"""

from pyocd.core.helpers import ConnectHelper

GPIOB = 0x50000400
MODER = GPIOB + 0x00
PUPDR = GPIOB + 0x0C
IDR = GPIOB + 0x10

PINS = {3: "PB3/SCL", 4: "PB4/SDA"}


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

        mod = r(MODER)
        for bit in PINS:
            mod &= ~(3 << (bit * 2))        # 00 = 输入
        w(MODER, mod)

        for pull, name in ((0b00, "无上下拉(浮空)"), (0b10, "内部下拉"), (0b01, "内部上拉")):
            p = r(PUPDR)
            for bit in PINS:
                p &= ~(3 << (bit * 2))
                p |= (pull << (bit * 2))
            w(PUPDR, p)

            samples = {b: [] for b in PINS}
            for _ in range(24):
                v = r(IDR)
                for bit in PINS:
                    samples[bit].append((v >> bit) & 1)

            print("[%s]" % name)
            for bit, label in PINS.items():
                s = samples[bit]
                ones = sum(s)
                tag = "稳定高" if ones == len(s) else ("稳定低" if ones == 0 else "抖(%d/%d 高)" % (ones, len(s)))
                print("   %-9s : %s" % (label, tag))

        t.resume()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
