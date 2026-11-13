#!/usr/bin/env python3
"""Read live IMU failure records using the matching ELF; never writes flash.

Run with the Python interpreter that has pyocd installed:
    python tools/imu_status.py
    python tools/imu_status.py --frequency 50000 --uid CZ_2023_6688
    python tools/imu_status.py --quick  # Verify 3 code samples; not a full-image check
"""
import argparse
import json
import struct
import sys
from pathlib import Path

from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

LAYOUT = struct.Struct("<4sIII7H8B14s16s128s16s4B")
PHASE = {1: "full address / identity scan", 2: "configure / data-register probe",
         3: "select read mode", 4: "calibration", 5: "ready", 6: "runtime"}
FIELDS = ("magic", "seq", "tick", "hal_error", "sr1", "sr2",
          "length", "failures", "calib_ok", "calib_nack", "calib_ff",
          "phase", "bus", "fault", "addr", "whoami", "reg", "op", "nack_stage", "frame",
          "ack_map", "id75", "id75_ok", "scan_count", "found_count", "scan_state", "reserved")


def decode(data):
    if len(data) != LAYOUT.size * 2:
        raise ValueError("Invalid IMU diagnostic record length")
    result = []
    for i in range(2):
        d = dict(zip(FIELDS, LAYOUT.unpack_from(data, i * LAYOUT.size)))
        if d["magic"] != b"EIMU":
            result.append({"bus": i, "initialized": False})
            continue
        if d["bus"] != i:
            raise ValueError("Diagnostic bus index does not match record")
        d["magic"] = d["magic"].decode("ascii")
        d["initialized"] = True
        d["phase_name"] = PHASE.get(d["phase"], "unknown")
        d["frame"] = d["frame"].hex(" ")
        ack_map = d.pop("ack_map")
        ids = d.pop("id75")
        valid = d.pop("id75_ok")
        d.pop("reserved")
        d["devices"] = []
        for addr in range(0x08, 0x78):
            if ack_map[addr >> 3] & (1 << (addr & 7)):
                ok = bool(valid[addr >> 3] & (1 << (addr & 7)))
                d["devices"].append({"address": f"0x{addr:02X}", "id75_read_ok": ok,
                                     "id75": f"0x{ids[addr]:02X}" if ok else None})
        result.append(d)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", type=Path, default=Path(__file__).resolve().parents[1] /
                    "EIDE/build/Debug/ciallo~.elf")
    ap.add_argument("--frequency", type=int, default=50000)
    ap.add_argument("--uid", default=None)
    ap.add_argument("--quick", action="store_true",
                    help="Compare 3 short code samples instead of the entire image")
    args = ap.parse_args()
    with args.elf.open("rb") as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name(".symtab")
        symbols = symtab.get_symbol_by_name("g_ear_imu_diag") if symtab else None
        if not symbols or symbols[0]["st_size"] != LAYOUT.size * 2:
            raise ValueError("ELF lacks the expected diagnostic array; rebuild firmware first")
        addr = symbols[0]["st_value"]
        if not (0x20000000 <= addr <= 0x20000C00 - LAYOUT.size * 2):
            raise ValueError("Diagnostic array is outside target RAM")
        segments = [(s["p_paddr"], s.data()) for s in elf.iter_segments()
                    if s["p_type"] == "PT_LOAD" and s["p_filesz"] and
                    0x08000000 <= s["p_paddr"] < 0x08006000]
        checks = [("load segment", base, expected) for base, expected in segments]
        if args.quick:
            checks = []
            for name in ("IMU_Scan", "IMU_BringUpOnBus", "Ear_FaultIndicate"):
                matches = symtab.get_symbol_by_name(name)
                if not matches:
                    raise ValueError("ELF lacks quick-check symbol: " + name)
                sym = matches[0]
                section = elf.get_section(sym["st_shndx"])
                base = sym["st_value"] & ~1
                offset = base - section["sh_addr"]
                n = min(sym["st_size"], 96)
                expected = section.data()[offset:offset + n]
                if n == 0 or len(expected) != n or not 0x08000000 <= base <= 0x08006000 - n:
                    raise ValueError("Invalid code sample: " + name)
                checks.append((name, base, expected))
    if not segments:
        raise ValueError("ELF has no target flash load segments")
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.uid, target_override="cortex_m", blocking=False,
        options={"frequency": args.frequency, "connect_mode": "attach",
                 "resume_on_disconnect": False})
    if session is None:
        raise RuntimeError("No debug probe found")
    with session:
        t = session.target
        running = t.get_state() == Target.State.RUNNING
        t.halt()
        try:
            # Capture the small RAM block first to shorten the critical read.
            data = bytes(t.read_memory_block8(addr, LAYOUT.size * 2))
            for name, base, expected in checks:
                actual = bytes(t.read_memory_block8(base, len(expected)))
                if actual != expected:
                    raise RuntimeError("Installed firmware differs from ELF (" + name +
                                       "); refusing to decode RAM")
        finally:
            if running:
                t.resume()
    print(json.dumps({"elf": str(args.elf), "address": hex(addr),
                      "firmware_verified": not args.quick,
                      "verification": "sampled_code_only" if args.quick else "full_image",
                      "checks": [name for name, _, _ in checks],
                      "records": decode(data)}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(type(exc).__name__ + ": " + str(exc), file=sys.stderr)
        sys.exit(1)
