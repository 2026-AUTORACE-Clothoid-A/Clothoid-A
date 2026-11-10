#!/usr/bin/env python3
"""Print run.sh / ablate.sh results: process CPU and per-callback times.

    python3 tools/load_profile/show.py <outdir>/results.jsonl
"""
import json
import sys

for line in open(sys.argv[1]):
    r = json.loads(line)
    print(f"== {r['label']:15s} CPU {r['cpu_pct']:5.1f}%")
    for n, c in r['callbacks'].items():
        if c['n'] < 2:
            continue
        print(f"   {n:22s} n={c['n']:5d} {c['hz']:6.1f}Hz mean {c['mean']:6.2f} p95 {c['p95']:6.2f}"
              f" max {c['max']:7.2f} ms  share {c['share_pct']:5.1f}%"
              f"  gap95 {c['gap_p95']:6.1f} gapmax {c['gap_max']:6.1f}")
