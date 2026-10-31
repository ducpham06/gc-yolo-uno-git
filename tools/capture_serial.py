#!/usr/bin/env python3
"""Capture the Yolo UNO benchmark output into results/ (CSV logs + summary table).

usage: python tools/capture_serial.py                       (auto-detect if one port is present)
       python tools/capture_serial.py --port COM7            (Windows)
       python tools/capture_serial.py --port /dev/ttyACM0    (Linux/macOS)
       python tools/capture_serial.py --list                 (show available ports)
       add --cmd b for benchmark-only runs (no trajectory logs, 5 repeats)
requires: pip install pyserial
"""
import argparse, os, time
import serial
from serial.tools import list_ports


def show_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("  (no serial ports found - is the board plugged in with a data-capable cable?)")
    for p in ports:
        print(f"  {p.device:12s} {p.description}  [{p.hwid}]")
    return ports


def parse_summary(line):
    parts = line.split()
    d = {"scenario": parts[1]}
    for kv in parts[2:]:
        k, v = kv.split("=", 1)
        d[k] = v
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port; auto-detected when exactly one is present")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="results")
    ap.add_argument("--cmd", default="r", help="r = runs with logs, b = benchmark only")
    ap.add_argument("--timeout", type=float, default=600)
    a = ap.parse_args()
    if a.list:
        show_ports()
        return
    if not a.port:
        ports = list(list_ports.comports())
        if len(ports) != 1:
            print("Pass --port explicitly. Available ports:")
            show_ports()
            raise SystemExit(1)
        a.port = ports[0].device
        print(f"using {a.port} ({ports[0].description})")
    os.makedirs(a.out, exist_ok=True)

    try:
        ser = serial.Serial(a.port, a.baud, timeout=1)
    except serial.SerialException as e:
        print(f"could not open {a.port}: {e}")
        print("Available ports:")
        show_ports()
        print("If the port is listed but busy, close any serial monitor (PlatformIO, Arduino IDE).")
        raise SystemExit(1)
    time.sleep(1.0)
    ser.reset_input_buffer()
    ser.write(a.cmd.encode())
    cur, summaries, info = None, [], []
    t0 = time.time()
    raw = open(os.path.join(a.out, "serial_raw.txt"), "w")
    while time.time() - t0 < a.timeout:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue
        raw.write(line + "\n")
        if line.startswith("#BEGIN"):
            cur = open(os.path.join(a.out, f"log_{line.split()[1]}.csv"), "w")
        elif line.startswith("#END"):
            if cur: cur.close()
            cur = None
        elif line.startswith("#SUMMARY"):
            summaries.append(parse_summary(line))
            print(line)
        elif line.startswith("#INFO"):
            info.append(line); print(line)
        elif line.startswith("#DONE"):
            break
        elif cur:
            cur.write(line + "\n")
    raw.close()
    if not summaries:
        raise SystemExit("no #SUMMARY lines received - check port / reset the board")

    keys = list(summaries[0].keys())
    with open(os.path.join(a.out, "summary.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for s in summaries:
            f.write(",".join(s.get(k, "") for k in keys) + "\n")
    cols = ["scenario", "scalar", "n_con", "total_avg_us", "total_p99_us", "total_max_us",
            "safety_avg_us", "qp_avg_us", "util_avg_pct", "qp_iter_avg", "fallbacks", "max_pen_mm", "final_err_rad"]
    with open(os.path.join(a.out, "summary.md"), "w") as f:
        for l in info:
            f.write(f"<!-- {l} -->\n")
        f.write("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n")
        for s in summaries:
            f.write("| " + " | ".join(s.get(c, "") for c in cols) + " |\n")
    print(f"saved to {a.out}/  (plot with: python tools/plot_results.py {a.out})")


if __name__ == "__main__":
    main()
