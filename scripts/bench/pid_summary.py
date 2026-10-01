#!/usr/bin/env python3
"""Score PID sweep runs: analyze.py JSON on stdin -> one table row per gain set (both wheels pooled)."""
import json
import math
import sys
from collections import defaultdict

import numpy as np


def pooled(rows, key):
    v = [r[key] for r in rows if r is not None and r.get(key) is not None and not math.isnan(r[key])]
    return v


text = sys.stdin.read()
runs = json.loads(text[text.index("\n[") + 1:])  # skip the entrypoint banner
by_gain = defaultdict(lambda: defaultdict(list))
for r in runs:
    g = r['gains']
    gk = (g.get('kp', 1.0), g.get('ki', 2.0))
    part = g.get('part', 'spin')
    for seg in r['wheel_loop']:
        for w in ('left', 'right'):
            x = seg[w]
            if x is None:
                continue
            band = 'low' if x['target_mps'] < 0.06 else 'mid'
            by_gain[gk][(part, band)].append(x)
        if seg['left'] and seg['right']:
            mm = seg['left']['ss_err_pct'] - seg['right']['ss_err_pct']
            by_gain[gk][(part, 'lr')].append(dict(lr=abs(mm)))

print(f"{'kp':>4} {'ki':>4} | {'mid ovs%':>8} {'settle s':>8} {'|ss|%':>6} {'rip%':>5} {'L-R%':>5} | "
      f"{'low |ss|%':>9} {'low rip%':>8} {'low ovs%':>8} {'stall':>5} n")
for gk in sorted(by_gain):
    d = by_gain[gk]
    mid = d[('spin', 'mid')] + d[('spin', 'low')]
    low = d[('low', 'low')] + d[('low', 'mid')]
    f = lambda rows, k, fn=np.mean: fn(pooled(rows, k)) if pooled(rows, k) else float('nan')
    print(f"{gk[0]:4.1f} {gk[1]:4.1f} | {f(mid, 'overshoot_pct'):8.1f} {f(mid, 'settle_s', np.median):8.2f} "
          f"{np.mean(np.abs(pooled(mid, 'ss_err_pct'))):6.1f} {f(mid, 'ripple_pct'):5.1f} "
          f"{f(d[('spin', 'lr')], 'lr'):5.1f} | {np.mean(np.abs(pooled(low, 'ss_err_pct'))) if low else float('nan'):9.1f} "
          f"{f(low, 'ripple_pct'):8.1f} {f(low, 'overshoot_pct'):8.1f} {f(low, 'stall_frac'):5.2f} {len(mid)}/{len(low)}")
