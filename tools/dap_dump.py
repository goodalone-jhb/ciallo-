#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通过 DAPLink 读走 PY32F002B 里的诊断快照（SWD，不需示波器也不需要万用表）。

固件在传感器起不来时会把"现场"持续写在 RAM 的 0x20000500，共 288 字节：

    0x00 magic  'EARD'      读到它就说明 RAM 里确实是本固件写的
    0x04 seq    刷新计数     两次读它不一样 = CPU 确实在跑
    0x08 tick   SysTick 毫秒
    0x0C fault  故障码（就是原来"抽几下"的那个数字）
    0x10 ack68 / who68 / ack69 / who69
    0x14 st68[4]  0x68 上 0x75/0x3B/0x41/0x1A 的读取状态
    0x18 st69[4]  0x69 同上
    0x1C nerr     0x00~0x7F 中读失败的个数
    0x20 reg[128] 0x00~0x7F 寄存器镜像
    0xA0 status[128] 每个寄存器的状态（0 成功 1 地址 NAK 2 寄存器 NAK 3 读地址 NAK）

用法：
    python tools/dap_dump.py              # 连一次就打印
    python tools/dap_dump.py -n 10        # 连不上就重试 10 次（每次间隔 3 秒）
    python tools/dap_dump.py -w 20        # 连续观察 20 秒，看 seq 是否在涨

依赖：pyOCD（已装）。探针必须贴着板子的 SWD 触点。
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

DBG_ADDR = 0x20000500
DBG_LEN = 288
MAGIC = b"EARD"

REG_NAMES = {
    0x19: "SMPLRT_DIV",
    0x1A: "CONFIG",
    0x1B: "GYRO_CONFIG",
    0x1C: "ACCEL_CONFIG",
    0x1D: "ACCEL_CONFIG2",
    0x1E: "LP_ACCEL_ODR",
    0x23: "FIFO_EN",
    0x37: "INT_PIN_CFG",
    0x38: "INT_ENABLE",
    0x3B: "ACCEL_XOUT_H",
    0x3D: "ACCEL_YOUT_H",
    0x3F: "ACCEL_ZOUT_H",
    0x41: "TEMP_OUT_H",
    0x43: "GYRO_XOUT_H",
    0x45: "GYRO_YOUT_H",
    0x47: "GYRO_ZOUT_H",
    0x6B: "PWR_MGMT_1",
    0x6C: "PWR_MGMT_2",
    0x75: "WHO_AM_I",
}

STAGE = {0: "OK", 1: "地址NAK", 2: "寄存器NAK", 3: "读地址NAK"}


def run_commander(commands, freq=1000000, timeout=45):
    """调用 pyocd commander 执行命令，返回 (成功?, 合并输出)。"""
    args = ["pyocd", "commander", "-t", "cortex_m", "-f", str(freq)]
    for c in commands:
        args += ["-c", c]
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "超时（%d 秒）" % timeout
    except FileNotFoundError:
        return False, "找不到 pyocd 命令，请先 pip install pyocd"
    out = (p.stdout or "") + (p.stderr or "")
    ok = ("Error" not in out) and (p.returncode == 0)
    return ok, out


def read_bytes(retries=1, freq=1000000):
    """把 288 字节读回来。优先用 save 落盘，失败退回 read8 解析。"""
    tmp = os.path.join(tempfile.gettempdir(), "ear_dbg.bin")

    for attempt in range(1, retries + 1):
        if os.path.exists(tmp):
            os.remove(tmp)

        ok, out = run_commander(["save %s 0x%08X %d" % (tmp, DBG_ADDR, DBG_LEN)], freq)
        if ok and os.path.exists(tmp):
            data = open(tmp, "rb").read()
            if len(data) >= DBG_LEN:
                return data[:DBG_LEN]
            out += "\n(save 只拿到 %d 字节)" % len(data)

        # 退路：read8 的文本输出
        ok2, out2 = run_commander(["read8 0x%08X %d" % (DBG_ADDR, DBG_LEN)], freq)
        parsed = parse_hexdump(out2)
        if parsed and len(parsed) >= DBG_LEN:
            return parsed[:DBG_LEN]

        brief = "\n".join(l for l in out.splitlines() if "E " in l or "Error" in l)[-400:]
        print("  尝试 %d/%d 失败：%s" % (attempt, retries, brief or "未知原因"))
        if attempt < retries:
            print("  请压住 DAPLink 的 SWD 触点，3 秒后重试……")
            time.sleep(3)

    return None


def parse_hexdump(text):
    """从 pyocd 的 read8 输出里抠出十六进制字节。"""
    out = bytearray()
    for line in text.splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        head = line.split(":", 1)[0].replace("0x", "")
        try:
            int(head, 16)
        except ValueError:
            continue
        body = line.split(":", 1)[1]
        for tok in body.replace("|", " ").split():
            tok = tok.strip()
            if len(tok) == 2:
                try:
                    out.append(int(tok, 16))
                except ValueError:
                    pass
    return bytes(out)


def u32(d, off):
    return int.from_bytes(d[off:off + 4], "little")


def decode(d):
    print("=" * 66)
    magic = d[0:4]
    print("magic      : %r %s" % (magic, "← 对上了，确实是本固件" if magic == MAGIC
                                  else "← 不对！RAM 里没有快照（固件没跑到这里？）"))
    if magic != MAGIC:
        print("原始前 64 字节：", d[:64].hex(" "))
        return
    print("seq        : %d" % u32(d, 4))
    print("tick       : %d ms" % u32(d, 8))
    print("fault      : %d（原来“抽几下”的那个数字）" % u32(d, 12))
    print("0x68       : %s  WHO_AM_I=0x%02X  st(75,3B,41,1A)=%s"
          % ("应答" if d[16] else "无应答", d[17], list(d[20:24])))
    print("0x69       : %s  WHO_AM_I=0x%02X  st(75,3B,41,1A)=%s"
          % ("应答" if d[18] else "无应答", d[19], list(d[24:28])))
    print("失败寄存器 : %d / 128" % d[28])

    reg = d[0x20:0x20 + 128]
    status = d[0xA0:0xA0 + 128]

    print("-" * 66)
    print("寄存器镜像（. = 读失败，值见下方状态表）：")
    print("      " + " ".join("%02X" % i for i in range(16)))
    for row in range(8):
        cells = []
        for col in range(16):
            i = row * 16 + col
            cells.append("%02X" % reg[i] if status[i] == 0 else "..")
        print("0x%02X: %s" % (row * 16, " ".join(cells)))

    bad = [(i, status[i]) for i in range(128) if status[i] != 0]
    if bad:
        print("-" * 66)
        print("读失败的寄存器：")
        for i, s in bad:
            print("  0x%02X %-14s %s" % (i, REG_NAMES.get(i, ""), STAGE.get(s, str(s))))
    else:
        print("-" * 66)
        print("128 个寄存器全部读取成功 —— 说明 I2C 通路是好的，问题不在总线上")

    print("-" * 66)
    key = [0x75, 0x1A, 0x1B, 0x1C, 0x6B, 0x6C, 0x3B, 0x3F, 0x41]
    for i in key:
        print("  0x%02X %-14s = 0x%02X (%3d)" % (i, REG_NAMES.get(i, ""), reg[i], reg[i]))
    print("=" * 66)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--retries", type=int, default=3, help="连接重试次数")
    ap.add_argument("-f", "--freq", type=int, default=1000000, help="SWD 时钟 Hz")
    ap.add_argument("-w", "--watch", type=int, default=0, help="连续观察秒数")
    args = ap.parse_args()

    if args.watch > 0:
        end = time.time() + args.watch
        prev_seq = None
        while time.time() < end:
            d = read_bytes(retries=2, freq=args.freq)
            if d and d[0:4] == MAGIC:
                seq = u32(d, 4)
                delta = "" if prev_seq is None else "（比上次 +%d）" % (seq - prev_seq)
                print("[%s] seq=%d tick=%d 失败寄存器=%d %s"
                      % (time.strftime("%H:%M:%S"), seq, u32(d, 8), d[28], delta))
                prev_seq = seq
            else:
                print("[%s] 没读到快照" % time.strftime("%H:%M:%S"))
            time.sleep(1.0)
        return

    d = read_bytes(retries=args.retries, freq=args.freq)
    if d is None:
        print("读不到。检查：DAPLink 是否压紧在 SWD 触点上 / 板子是否上电 / 是否被别的软件占用。")
        sys.exit(1)
    decode(d)


if __name__ == "__main__":
    main()
