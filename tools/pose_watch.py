#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时看姿态与舵机输出（SWD，只读，不停机跑固件）。

固件每 10ms 把 6 个 float 写进 g_ear_pose（RAM，24 字节，volatile）：
    pitch_tr    点头瞬态量（度）—— 驱动 CH1
    yaw_deg     转头积分角（度）—— 驱动 CH2
    pitch_rate  俯仰角速度 deg/s
    yaw_rate    转头角速度 deg/s
    out1        CH1 实际脉宽 us
    out2        CH2 实际脉宽 us

用法（必须用装了 pyocd 的那个解释器）：
    "C:/Users/goodalone/AppData/Local/Microsoft/WindowsApps/python.exe" tools/pose_watch.py
    ... tools/pose_watch.py -n 200            # 采 200 行后退出
    ... tools/pose_watch.py -i 0.05           # 50ms 一行
    ... tools/pose_watch.py --uid CZ_2023_6688

怎么用它判断故障：
    · 转头时 yaw_rate 有几百 dps、yaw_deg 跟着涨，但 out2 一直是 1500
      -> 算法/参数问题（看 TURN_* 那几个宏）
    · 转头时 yaw_rate 一直是 0 或只有个位数
      -> 传感器这一轴没数据：贴片方向不对，或者 Z 轴根本没朝上
    · 不动时 yaw_deg 一直往一个方向爬
      -> 零偏没标干净（上电时板子被碰了）

注意：每次采样都会短暂 halt 内核再 resume，舵机输出会掉一帧左右，
      所以观感上可能有极轻微的顿挫，这是工具造成的，不是固件行为。
"""

import argparse
import struct
import sys
import time
from pathlib import Path

from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

POSE = struct.Struct("<6f")
FIELDS = ("pitch_tr", "yaw_deg", "pitch_rate", "yaw_rate", "out1", "out2")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", type=Path, default=Path(__file__).resolve().parents[1] /
                    "EIDE/build/Debug/ciallo~.elf")
    ap.add_argument("-n", "--count", type=int, default=0, help="0 = 一直采")
    ap.add_argument("-i", "--interval", type=float, default=0.1, help="采样间隔（秒）")
    ap.add_argument("--frequency", type=int, default=50000)
    ap.add_argument("--uid", default=None)
    args = ap.parse_args()

    with args.elf.open("rb") as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name(".symtab")
        got = symtab.get_symbol_by_name("g_ear_pose") if symtab else None
        if not got:
            raise SystemExit("ELF 里没有 g_ear_pose，先重新编译固件")
        sym = got[0]
        if sym["st_size"] != POSE.size:
            raise SystemExit("g_ear_pose 大小是 %d，期望 %d —— 固件和脚本不是同一版"
                             % (sym["st_size"], POSE.size))
        addr = sym["st_value"]
        if not 0x20000000 <= addr <= 0x20000C00 - POSE.size:
            raise SystemExit("g_ear_pose 不在目标 RAM 里：0x%08X" % addr)

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.uid, target_override="cortex_m", blocking=False,
        options={"frequency": args.frequency, "connect_mode": "attach",
                 "resume_on_disconnect": True})
    if session is None:
        raise SystemExit("没找到调试探针（检查 DAPLink 有没有贴住 SWD 触点）")

    print("g_ear_pose @ 0x%08X   Ctrl-C 结束" % addr)
    print("%8s %8s %9s %9s %8s %8s" % ("pitch_tr", "yaw_deg", "p_rate", "y_rate", "out1", "out2"))

    n = 0
    with session:
        t = session.target
        while args.count == 0 or n < args.count:
            running = t.get_state() == Target.State.RUNNING
            t.halt()
            try:
                raw = bytes(t.read_memory_block8(addr, POSE.size))
            finally:
                if running:
                    t.resume()
            v = POSE.unpack(raw)
            print("%8.1f %8.1f %9.0f %9.0f %8.0f %8.0f" % v)
            sys.stdout.flush()
            n += 1
            time.sleep(args.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
