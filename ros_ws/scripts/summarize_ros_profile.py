#!/usr/bin/env python3
"""Summarize the process snapshots written by profile_ros_load.sh."""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} PREFIX", file=sys.stderr)
        return 2
    source = Path(sys.argv[1] + ".ps.txt")
    if not source.is_file():
        print(f"missing {source}", file=sys.stderr)
        return 1

    samples: dict[str, list[float | int | str]] = defaultdict(
        lambda: [0.0, 0.0, 0, 0, 0, ""]
    )
    for line in source.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("PID "):
            continue
        fields = line.split(None, 8)
        if len(fields) < 9:
            continue
        try:
            pid = fields[0]
            threads = int(fields[2])
            cpu = float(fields[3])
            rss = int(fields[5])
        except ValueError:
            continue
        row = samples[pid]
        row[0] = float(row[0]) + cpu
        row[1] = max(float(row[1]), cpu)
        row[2] = int(row[2]) + 1
        row[3] = max(int(row[3]), rss)
        row[4] = max(int(row[4]), threads)
        row[5] = fields[8]

    writer = csv.writer(sys.stdout)
    writer.writerow(["pid", "avg_cpu_percent", "max_cpu_percent", "samples",
                     "max_rss_kb", "max_threads", "command"])
    rows = []
    for pid, (total, peak, count, rss, threads, command) in samples.items():
        if count:
            rows.append((float(peak), pid, total / count, peak, count, rss, threads, command))
    for _, pid, average, peak, count, rss, threads, command in sorted(rows, reverse=True):
        writer.writerow([pid, f"{average:.2f}", f"{peak:.2f}", count, rss, threads, command])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
