#!/usr/bin/env python3
"""Per-test metrics from the bench recordings. Usage: analyze.py <npz-or-prefix> ... -> JSON to stdout."""

import glob
import json
import math
import sys

import os

import numpy as np

BENCH = os.environ.get('BENCH_DIR', '/root/bench')

SEP, RAD = 0.250, 0.0308
MRPS = 2 * math.pi / 1000.0 * RAD  # m/s per mrps at the wheel rim
TICK_MRPS = 1000.0 / 515 / 0.1  # one tick in a 100 ms window


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load(prefix):
    f = (
        prefix
        if prefix.endswith('.npz')
        else sorted(
            x for x in glob.glob(f'{BENCH}/out/{prefix}_*.npz') if 'clouds' not in x
        )[-1]
    )
    d = dict(np.load(f))
    m = json.load(open(f.replace('.npz', '.json')))
    return d, m, f


def segments(m, start_prefix, stop_prefix):
    ev = m['events']
    out = []
    for i, (t, label) in enumerate(ev):
        if label.startswith(start_prefix) and not label.startswith(stop_prefix):
            for t2, l2 in ev[i + 1 :]:
                if l2.startswith(stop_prefix) or l2.startswith(start_prefix):
                    out.append((t, t2, label))
                    break
    return out


def win(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)]


def step_metrics(it, t0, t1, col_tgt, col_fw, col_meas, col_pwm):
    """Wheel loop metrics vs the post-limiter target. t0 = raw command step, t1 = stop command."""
    seg = win(it, t0 - 0.2, t1)
    t, tgt, fw, meas, pwm = (
        seg[:, 0],
        seg[:, col_tgt],
        seg[:, col_fw],
        seg[:, col_meas],
        seg[:, col_pwm],
    )
    final = tgt[-1]
    if abs(final) < 1:
        return None
    sgn = np.sign(final)
    tgt, fw, meas, pwm = tgt * sgn, fw * sgn, meas * sgn, pwm * sgn
    final = abs(final)
    reach = t[np.argmax(tgt >= final * 0.999)]  # ramp end (limiter)
    nz = t[np.argmax(tgt > 1)]
    moving = (fw > 1) & (t >= nz)
    first_move = t[np.argmax(moving)] if moving.any() else float('nan')
    # the 100 ms tick window quantises speed to ~19 mrps: judge overshoot/settling on a 200 ms mean
    fwm = np.convolve(fw, np.ones(20) / 20, mode='same')
    # lag: shift fw against tgt during the ramp to minimise the squared error
    ramp = (t >= nz) & (t <= reach + 0.3)
    lags = np.arange(0, 0.4, 0.01)
    errs = (
        [np.mean((np.interp(t[ramp], t - L, fw) - tgt[ramp]) ** 2) for L in lags]
        if ramp.sum() > 5
        else [0]
    )
    lag = float(lags[int(np.argmin(errs))])
    after = t >= reach
    inner = after & (t <= t[-1] - 0.1)  # the 200 ms mean is skewed at the segment end
    over = (
        float((fwm[inner].max() - final) / final * 100) if inner.any() else float('nan')
    )
    # settling: last time the 200 ms mean leaves the max(5 %, half a tick) band after the ramp end
    band = max(0.05 * final, TICK_MRPS * 0.5)
    out_band = np.where(inner & (np.abs(fwm - final) > band))[0]
    settle = float(t[out_band[-1]] - reach) if len(out_band) else 0.0
    if len(out_band) and t[out_band[-1]] > t[-1] - 0.25:
        settle = float('nan')
    ss = t >= max(reach + 0.6, t[-1] - 1.2)
    ss_mean_fw = float(np.mean(fw[ss])) if ss.any() else float('nan')
    ss_mean_meas = float(np.mean(meas[ss])) if ss.any() else float('nan')
    rise_ref = (
        float(t[np.argmax(fw >= 0.9 * final)] - t0)
        if (fw >= 0.9 * final).any()
        else float('nan')
    )
    return dict(
        target_mrps=float(final),
        target_mps=float(final * MRPS),
        ramp_s=float(reach - t0),
        deadtime_s=float(first_move - nz),
        lag_s=lag,
        overshoot_pct=over,
        settle_s=settle,
        rise90_from_cmd_s=rise_ref,
        ss_err_pct=(ss_mean_fw - final) / final * 100,
        ss_err_host_pct=(ss_mean_meas - final) / final * 100,
        ripple_pct=float(np.std(fw[ss]) / final * 100) if ss.any() else float('nan'),
        pwm_ss=float(np.mean(pwm[ss])) if ss.any() else float('nan'),
        pwm_max=float(pwm.max()),
        stall_frac=float(np.mean(fw[ss] < 1)) if ss.any() else float('nan'),
    )


def wheel_loop(d, m, start, stop):
    it = d['intro']
    res = []
    for t0, t1, label in segments(m, start, stop):
        L = step_metrics(it, t0, t1, 1, 3, 2, 4)
        R = step_metrics(it, t0, t1, 5, 7, 6, 8)
        res.append(dict(label=label, t0=t0, t1=t1, left=L, right=R))
    return res


def body_response(d, m, start, stop, kind):
    """Body level: raw cmd step -> wheel-odom twist (what the EKF and Nav2 see)."""
    wo = d['wheel']
    ekf = d['ekf']
    out = []
    for t0, t1, label in segments(m, start, stop):
        cmdv = float(label.split()[-1])
        col = 4 if kind == 'v' else 5
        s = win(wo, t0, t1)
        if len(s) < 5 or abs(cmdv) < 1e-6:
            continue
        y = s[:, col] / cmdv
        t = s[:, 0] - t0
        r90 = float(t[np.argmax(y >= 0.9)]) if (y >= 0.9).any() else float('nan')
        ss = t > t[-1] - 1.0
        out.append(
            dict(
                label=label,
                cmd=cmdv,
                rise90_s=r90,
                ss_ratio=float(np.mean(y[ss])),
                overshoot_pct=float((y.max() - 1) * 100),
                ekf_ss_ratio=float(np.mean(win(ekf, t1 - 1.0, t1)[:, col]) / cmdv),
            )
        )
    return out


def latency(d, m, start):
    cmd, co, it, js = d['cmd'], d['cmd_out'], d['intro'], d['js']
    out = []
    for t0, label in m['events']:
        if not label.startswith(start):
            continue
        a = cmd[(cmd[:, 0] >= t0 - 0.05)]
        tc = a[np.argmax((np.abs(a[:, 1]) > 1e-4) | (np.abs(a[:, 2]) > 1e-4)), 0]
        b = co[co[:, 0] >= tc]
        to = b[np.argmax((np.abs(b[:, 1]) > 1e-4) | (np.abs(b[:, 2]) > 1e-4)), 0]
        c = it[it[:, 0] >= tc]
        tt = c[np.argmax(np.abs(c[:, 1]) > 0.5), 0]
        tf = c[np.argmax(np.abs(c[:, 3]) > 0.5), 0]
        j = js[js[:, 0] >= tc]
        p0 = j[0, 1:3]
        tj = j[np.argmax(np.any(np.abs(j[:, 1:3] - p0) > 1e-4, axis=1)), 0]
        out.append(
            dict(
                label=label,
                cmd_to_limiter_ms=(to - tc) * 1000,
                cmd_to_target_ms=(tt - tc) * 1000,
                cmd_to_first_tick_ms=(tj - tc) * 1000,
                cmd_to_fw_speed_ms=(tf - tc) * 1000,
            )
        )
    return out


def rel(a, b):
    """Pose b expressed in the frame of pose a."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])


def scan_to_scan(clouds, a, b, init):
    """Pose of keyframe b in keyframe a's frame by ICP between their own clouds (no global map)."""
    from bench import icp, normals, voxel

    key = {k.split('_', 1)[1]: k for k in clouds.files}
    if a not in key or b not in key:
        return None
    ref = voxel(clouds[key[a]], 0.01)
    n, good = normals(ref)
    T, rmse, inl = icp(voxel(clouds[key[b]], 0.02), ref[good], n[good], tuple(init))
    return np.array(T), rmse, inl


def kf_pairs(m, pairs, clouds=None):
    k = {x['label']: x for x in m['keyframes']}
    out = []
    for a, b in pairs:
        if a not in k or b not in k:
            continue
        A, B = k[a], k[b]
        tr, ek, wh = (
            rel(A['truth'], B['truth']),
            rel(A['ekf'], B['ekf']),
            rel(A['wheel'], B['wheel']),
        )
        s2s = scan_to_scan(clouds, a, b, tr) if clouds is not None else None
        if s2s is not None and s2s[2] > 0.75:
            tr = s2s[0]
        dist = float(np.hypot(tr[0], tr[1]))
        out.append(
            dict(
                a=a,
                b=b,
                truth=tr.tolist(),
                ekf=ek.tolist(),
                wheel=wh.tolist(),
                truth_dist=dist,
                ekf_err_xy_m=float(np.hypot(*(ek[:2] - tr[:2]))),
                wheel_err_xy_m=float(np.hypot(*(wh[:2] - tr[:2]))),
                ekf_dist_err_pct=float((np.hypot(ek[0], ek[1]) - dist) / dist * 100)
                if dist > 0.05
                else None,
                wheel_dist_err_pct=float((np.hypot(wh[0], wh[1]) - dist) / dist * 100)
                if dist > 0.05
                else None,
                ekf_yaw_err_deg=float(math.degrees(wrap(ek[2] - tr[2]))),
                wheel_yaw_err_deg=float(math.degrees(wrap(wh[2] - tr[2]))),
                icp_rmse_mm=[A['rmse'] * 1000, B['rmse'] * 1000],
                truth_src='scan-to-scan'
                if s2s is not None and s2s[2] > 0.75
                else 'global',
                s2s_fit=[s2s[1] * 1000, s2s[2]] if s2s is not None else None,
                imu_yaw_err_deg=float(
                    math.degrees(wrap(wrap(B['imu'] - A['imu']) - tr[2]))
                ),
            )
        )
    return out


def unwrapped_turn(arr, col, t0, t1):
    s = win(arr, t0, t1)
    return float(np.unwrap(s[:, col])[-1] - np.unwrap(s[:, col])[0])


def run(name):
    d, m, f = load(name)
    cf = f.replace('.npz', '_clouds.npz')
    clouds = np.load(cf) if os.path.exists(cf) else None
    test = m['name']
    R = dict(file=f, test=test, abort=m['abort'])
    if test == 'spin':
        R['wheel_loop'] = wheel_loop(d, m, 'spin step', 'spin stop')
        R['body'] = body_response(d, m, 'spin step', 'spin stop', 'w')
        R['latency'] = latency(d, m, 'spin step')
        R['turns'] = []
        for t0, t1, label in segments(m, 'spin step', 'spin stop'):
            t1b = t1 + 1.15
            R['turns'].append(
                dict(
                    label=label,
                    imu=unwrapped_turn(d['imu'], 1, t0, t1b),
                    ekf=unwrapped_turn(d['ekf'], 3, t0, t1b),
                    wheel=unwrapped_turn(d['wheel'], 3, t0, t1b),
                    cmd=float(
                        np.trapz(
                            win(d['cmd_out'], t0, t1b)[:, 2],
                            win(d['cmd_out'], t0, t1b)[:, 0],
                        )
                    ),
                )
            )
        R['kf'] = kf_pairs(m, [('start', 'end')])
    elif test == 'lin':
        R['wheel_loop'] = wheel_loop(d, m, 'lin step', 'lin stop')
        R['body'] = body_response(d, m, 'lin step', 'lin stop', 'v')
        R['latency'] = latency(d, m, 'lin step')
        R['kf'] = kf_pairs(
            m, [(f'lin{v}_a', f'lin{v}_b') for v in (0.1, 0.2, 0.3)], clouds
        )
    elif test == 'low':
        R['wheel_loop'] = wheel_loop(d, m, 'low', 'low stop')
        R['wheel_loop'] = [
            x for x in R['wheel_loop'] if not x['label'].startswith('low stop')
        ]
        R['body'] = body_response(d, m, 'low lin', 'low stop', 'v') + body_response(
            d, m, 'low ang', 'low stop', 'w'
        )
    elif test == 'arc':
        R['wheel_loop'] = wheel_loop(d, m, 'arc ', 'arc stop')
        out = []
        for t0, t1, label in segments(m, 'arc ', 'arc stop'):
            v, w = map(float, label.split()[1:3])
            wo = win(d['wheel'], t1 - 2.0, t1)
            im = win(d['imu'], t1 - 2.0, t1)
            ek = win(d['ekf'], t1 - 2.0, t1)
            vw, ww = float(np.mean(wo[:, 4])), float(np.mean(wo[:, 5]))
            out.append(
                dict(
                    label=label,
                    cmd_v=v,
                    cmd_w=w,
                    cmd_radius=v / w,
                    odom_v=vw,
                    odom_w=ww,
                    imu_w=float(np.mean(im[:, 2])) / 1.03,
                    ekf_v=float(np.mean(ek[:, 4])),
                    ekf_w=float(np.mean(ek[:, 5])),
                    achieved_radius=vw / ww,
                    outer_wheel_cmd_mps=v + w * SEP / 2,
                )
            )
        R['arc'] = out
        R['kf'] = kf_pairs(
            m,
            [(f'arc{v}_{w}_a', f'arc{v}_{w}_b') for v, w in ((0.3, 1.2), (0.35, 1.4))],
        )
    elif test == 'rev':
        R['kf'] = kf_pairs(m, [('rev_a', 'rev_b'), ('rev_b', 'rev_c')])
        it = d['intro']
        rv = []
        for t0, label in m['events']:
            if not (label.startswith('rev back') or label.startswith('rev spin -')):
                continue
            # time from the target sign flip to the wheel moving the new way (left wheel)
            s = win(it, t0, t0 + 1.3)
            for name_, ct, cf in (('left', 1, 3), ('right', 5, 7)):
                sg = np.sign(s[-1, ct])
                flip = s[np.argmax(np.sign(s[:, ct]) == sg), 0]
                mv = s[
                    (s[:, 0] >= flip)
                    & (np.sign(s[:, cf]) == sg)
                    & (np.abs(s[:, cf]) > 1)
                ]
                rv.append(
                    dict(
                        label=label,
                        wheel=name_,
                        flip_to_motion_ms=float((mv[0, 0] - flip) * 1000)
                        if len(mv)
                        else None,
                    )
                )
        R['reversal'] = rv
    elif test == 'rot':
        R['kf'] = kf_pairs(m, [('start', 'rot2'), ('rot2', 'rot-2')])
        R['turns'] = []
        kfs = {k['label']: k['t'] for k in m['keyframes']}
        for a, b in (('start', 'rot2'), ('rot2', 'rot-2')):
            R['turns'].append(
                dict(
                    seg=f'{a}->{b}',
                    imu=unwrapped_turn(d['imu'], 1, kfs[a], kfs[b]),
                    ekf=unwrapped_turn(d['ekf'], 3, kfs[a], kfs[b]),
                    wheel=unwrapped_turn(d['wheel'], 3, kfs[a], kfs[b]),
                    gyro=float(
                        np.trapz(
                            win(d['imu'], kfs[a], kfs[b])[:, 2],
                            win(d['imu'], kfs[a], kfs[b])[:, 0],
                        )
                    ),
                )
            )
    elif test == 'umb':
        k = [x['label'] for x in m['keyframes']]
        pairs = [
            (label, label.replace('_start', '_end'))
            for label in k
            if label.endswith('_start') and label.replace('_start', '_end') in k
        ]
        R['kf'] = kf_pairs(m, pairs, clouds)
    elif test == 'pid':
        R['gains'] = m['extra']
        R['wheel_loop'] = [
            x
            for x in wheel_loop(d, m, 'spin step', 'spin stop')
            + wheel_loop(d, m, 'low', 'low stop')
            if not x['label'].startswith('low stop')
        ]
    elif test == 'static':
        R['kf'] = kf_pairs(m, [('start', 'end')])
        e, w = d['ekf'], d['wheel']
        R['ekf_drift_m'] = float(np.hypot(e[-1, 1] - e[0, 1], e[-1, 2] - e[0, 2]))
        R['ekf_yaw_drift_deg'] = float(math.degrees(wrap(e[-1, 3] - e[0, 3])))
        R['wheel_drift_m'] = float(np.hypot(w[-1, 1] - w[0, 1], w[-1, 2] - w[0, 2]))
        R['imu_yaw_drift_deg'] = float(
            math.degrees(wrap(d['imu'][-1, 1] - d['imu'][0, 1]))
        )
        R['duration_s'] = float(e[-1, 0] - e[0, 0])
    else:
        R['kf'] = kf_pairs(m, [('start', 'end')])
    return R


if __name__ == '__main__':
    print(json.dumps([run(a) for a in sys.argv[1:]], indent=1, default=float))
