#!/usr/bin/env python3
"""
stall_detector.py

Post-process INA219 shunt-current captures from the pick-and-place cell.
The INA219 sits on the 12 V bus feeding the four stepper drivers, so what we
see is aggregate draw. A stall shows up as a fast rise in current that the
low-pass filter has not caught up with yet, i.e. the median-filtered trace
crosses a threshold derived from the idle baseline.

Usage:
    python scripts/stall_detector.py data/ina219_shunt_capture.csv
"""

import argparse
import csv
import statistics
import sys

BASELINE_WINDOW = 25        # samples used to estimate idle current
MEDIAN_KERNEL = 5           # odd, removes single-sample SPI glitches
LOWPASS_ALPHA = 0.25        # EMA weight on the newest sample
STALL_DELTA_MA = 180.0      # rise above baseline that counts as a stall
STALL_HOLD_SAMPLES = 3      # must persist to reject inrush from a step


def read_capture(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for rec in csv.DictReader(fh):
            rows.append(
                {
                    "t": float(rec["timestamp_s"]),
                    "joint": int(rec["joint_id"]),
                    "bus_v": float(rec["bus_voltage_v"]),
                    "ma": float(rec["shunt_current_ma"]),
                    "flag": int(rec["stall_flag"]),
                }
            )
    rows.sort(key=lambda r: (r["t"], r["joint"]))
    return rows


def median_filter(values, kernel=MEDIAN_KERNEL):
    """Centred median filter with edge clamping. Kernel must be odd."""
    if kernel % 2 == 0:
        raise ValueError("median kernel must be odd")
    half = kernel // 2
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append(statistics.median(values[lo:hi]))
    return out


def lowpass(values, alpha=LOWPASS_ALPHA):
    """First-order exponential smoothing. Seeded with the first sample."""
    if not values:
        return []
    out = [values[0]]
    for v in values[1:]:
        out.append(out[-1] + alpha * (v - out[-1]))
    return out


def baseline(series):
    head = series[: min(BASELINE_WINDOW, len(series))]
    return statistics.median(head) if head else 0.0


def detect(rows, delta_ma=STALL_DELTA_MA):
    """Return a list of (index, joint, rise_ma) stall events."""
    events = []
    for joint in sorted({r["joint"] for r in rows}):
        trace = [r["ma"] for r in rows if r["joint"] == joint]
        if len(trace) < STALL_HOLD_SAMPLES:
            continue

        smooth = lowpass(median_filter(trace))
        idle = baseline(smooth)
        threshold = idle + delta_ma

        run = 0
        for idx, value in enumerate(smooth):
            if value > threshold:
                run += 1
                if run == STALL_HOLD_SAMPLES:
                    events.append((idx, joint, value - idle))
            else:
                run = 0
    return events


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", help="INA219 CSV log")
    ap.add_argument(
        "--delta",
        type=float,
        default=STALL_DELTA_MA,
        help=f"stall threshold rise in mA (default {STALL_DELTA_MA})",
    )
    args = ap.parse_args(argv)

    rows = read_capture(args.capture)
    if not rows:
        print("empty capture", file=sys.stderr)
        return 1

    events = detect(rows, args.delta)
    flagged = sum(r["flag"] for r in rows)

    print(f"samples           : {len(rows)}")
    print(f"duration_s        : {rows[-1]['t'] - rows[0]['t']:.2f}")
    print(f"hardware flags    : {flagged}")
    print(f"detected stalls   : {len(events)}")
    for idx, joint, rise in events:
        print(f"  joint {joint}  sample {idx:>4}  rise {rise:7.1f} mA")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
