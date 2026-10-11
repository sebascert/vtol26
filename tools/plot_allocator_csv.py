#!/usr/bin/env python3
import csv
import sys
from pathlib import Path


def load_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(rows, key):
    return [float(r[key]) for r in rows if key in r and r[key] != ""]


def main():
    if len(sys.argv) != 2:
        print("usage: python tools\\plot_allocator_csv.py <allocator.csv>")
        return 1

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f"file not found: {csv_path}")
        return 2

    rows = load_rows(csv_path)
    if not rows:
        print("empty csv")
        return 3

    timestamps = to_float(rows, "timestamp_us")
    fitness = to_float(rows, "best_cost")
    cycles = to_float(rows, "cycles")

    print(f"samples={len(rows)}")
    print(f"fitness min={min(fitness):.6f} max={max(fitness):.6f}")
    print(f"cycles min={min(cycles):.0f} max={max(cycles):.0f}")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed. install: pip install matplotlib")
        return 0

    t0 = timestamps[0]
    t = [(x - t0) * 1e-6 for x in timestamps]

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax[0].plot(t, fitness, label="best_cost")
    ax[0].set_ylabel("fitness")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend()

    ax[1].plot(t, cycles, label="cycles", color="tab:orange")
    ax[1].set_ylabel("cycles")
    ax[1].set_xlabel("time [s]")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend()

    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

