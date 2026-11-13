#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 hex 和芯片 flash 里的实际内容逐字节比对，确认"到底烧进去了没有"。

用法:
    python tools/verify_flash.py                     # 比对 EIDE/build/Debug/ciallo~.hex
    python tools/verify_flash.py --hex 别的.hex
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_HEX = os.path.join(ROOT, "EIDE", "build", "Debug", "ciallo~.hex")

FLASH_BASE = 0x08000000


def parse_hex(path):
    """返回 (base, bytearray)：把 Intel HEX 摊平成连续镜像。"""
    seg = {}          # addr -> byte
    upper = 0
    lo, hi = None, None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue
            n = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            typ = int(line[7:9], 16)
            data = bytes.fromhex(line[9:9 + n * 2])
            if typ == 0x00:
                a = upper + addr
                for i, b in enumerate(data):
                    seg[a + i] = b
                lo = a if lo is None else min(lo, a)
                hi = (a + n) if hi is None else max(hi, a + n)
            elif typ == 0x01:
                break
            elif typ == 0x02:
                upper = int.from_bytes(data, "big") << 4
            elif typ == 0x04:
                upper = int.from_bytes(data, "big") << 16
    img = bytearray(hi - lo)
    for a, b in seg.items():
        img[a - lo] = b
    return lo, img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hex", default=DEFAULT_HEX)
    ap.add_argument("--base", type=lambda s: int(s, 0), default=FLASH_BASE)
    args = ap.parse_args()

    if not os.path.exists(args.hex):
        print("找不到 hex：%s" % args.hex)
        return 1

    lo, img = parse_hex(args.hex)
    print("hex      : %s" % os.path.basename(args.hex))
    print("建立时间 : %s" % __import__("time").strftime(
        "%Y-%m-%d %H:%M:%S", __import__("time").localtime(os.path.getmtime(args.hex))))
    print("镜像范围 : 0x%08X + %d 字节" % (args.base, len(img)))

    try:
        import pyocd.core.helpers as h
    except ImportError:
        print("这个 python 里没有 pyocd，换 WindowsApps 那个解释器")
        return 1

    session = h.ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 1000000},
        blocking=False, return_first=True)
    if session is None:
        print("没找到调试探针")
        return 1

    with session:
        target = session.target
        target.halt()
        try:
            got = bytes(target.read_memory_block8(args.base, len(img)))
        finally:
            pass

        diffs = [i for i in range(len(img)) if got[i] != img[i]]
        if not diffs:
            print("\n结果     : 芯片内容与 hex 完全一致（%d / %d 字节）" % (len(img), len(img)))
            print("           -> 新固件确实在芯片里，日志里的 programmed 0 bytes 属于正常跳过")
        else:
            print("\n结果     : 有 %d 处不同，第一处偏移 0x%X (hex=0x%02X chip=0x%02X)"
                  % (len(diffs), diffs[0], img[diffs[0]], got[diffs[0]]))
            show = diffs[:12]
            for i in show:
                print("   0x%08X  hex=0x%02X  chip=0x%02X"
                      % (args.base + i, img[i], got[i]))
            if len(diffs) > len(show):
                print("   ... 其余 %d 处省略" % (len(diffs) - len(show)))
            print("           -> 芯片里跑的不是这份 hex，需要重新烧录")

        target.resume()
    return 0


if __name__ == "__main__":
    sys.exit(main())
