#!/usr/bin/env python3
"""Kestrel base benchmark harness: safety layer, lidar-ICP ground truth, motion primitives, tests.

Run through run.sh (sidecar container, data on the host):  scripts/bench/run.sh bench <test> [k=v ...]
Ground truth = settled LD06 scans (transformed into base_link via TF) registered against one fixed
reference scan taken at the start pose (ref.npz). Results go to $BENCH_DIR/out/<test>_<ts>.
The arena frame is the robot pose when `reference` ran; FENCE is the free rectangle for the axle centre.
"""

import json
import math
import os
import signal
import sys
import threading
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue

BENCH = os.environ.get('BENCH_DIR', '/root/bench')
REF = f'{BENCH}/ref.npz'
STATE = f'{BENCH}/state.json'
# Free floor 0.5 m ahead, 1 m behind, 0.5 m to each side of the start pose, minus the footprint.
FENCE = tuple(
    float(v) for v in os.environ.get('FENCE', '-0.85,0.35,-0.35,0.35').split(',')
)
CENTRE = (-0.25, 0.0)
FOOTPRINT_R = 0.17  # chassis is a 0.15 m disc on the axle centre, plus margin
INTRO = [
    'left_wheel.target_velocity',
    'left_wheel.measured_velocity',
    'left_wheel.firmware_velocity',
    'left_wheel.pwm',
    'right_wheel.target_velocity',
    'right_wheel.measured_velocity',
    'right_wheel.firmware_velocity',
    'right_wheel.pwm',
]


class Abort(Exception):
    pass


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (x + c * b[0] - s * b[1], y + s * b[0] + c * b[1], wrap(t + b[2]))


def rel(a, b):
    return compose(inverse(a), b)


def inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, -t)


def tf_pts(p, T):
    c, s = math.cos(T[2]), math.sin(T[2])
    return p @ np.array([[c, s], [-s, c]]) + np.array([T[0], T[1]])


def voxel(p, res):
    if len(p) == 0:
        return p
    k = np.floor(p / res).astype(np.int64)
    _, idx = np.unique(k[:, 0] * 1000003 + k[:, 1], return_index=True)
    return p[np.sort(idx)]


def normals(ref, k=7):
    d2 = ((ref[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
    nn = np.argsort(d2, axis=1)[:, :k]
    n = np.zeros_like(ref)
    good = np.zeros(len(ref), bool)
    for i in range(len(ref)):
        q = ref[nn[i]]
        q = q - q.mean(0)
        w, v = np.linalg.eigh(q.T @ q)
        n[i] = v[:, 0]
        good[i] = (
            w[1] > 1e-9
            and w[0] / max(w[1], 1e-12) < 0.15
            and d2[i, nn[i, -1]] < 0.08**2
        )
    return n, good


def icp(src, ref, refn, init, iters=60):
    T = tuple(init)
    for i in range(iters):
        p = tf_pts(src, T)
        d2 = ((p[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(p)), j])
        thr = max(0.04, 0.4 * 0.85**i)
        m = dist < min(
            thr, max(0.015, np.percentile(dist, 75))
        )  # trimmed: people/chairs move
        if m.sum() < 30:
            return T, 1.0, 0.0
        pm, q, n = p[m], ref[j[m]], refn[j[m]]
        A = np.column_stack([n[:, 0], n[:, 1], n[:, 1] * pm[:, 0] - n[:, 0] * pm[:, 1]])
        b = -((pm - q) * n).sum(1)
        dx, dy, dth = np.linalg.lstsq(A, b, rcond=None)[0]
        T = compose((dx, dy, dth), T)
        if i > 15 and abs(dx) < 1e-5 and abs(dy) < 1e-5 and abs(dth) < 1e-5:
            break
    p = tf_pts(src, T)
    d2 = ((p[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
    j = d2.argmin(1)
    res = np.abs(((p - ref[j]) * refn[j]).sum(1))
    inl = np.sqrt(d2[np.arange(len(p)), j]) < 0.05
    return (
        T,
        float(np.sqrt(np.mean(res[inl] ** 2))) if inl.any() else 1.0,
        float(inl.mean()),
    )


class Bench(Node):
    def __init__(self):
        super().__init__('kestrel_bench')
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.rec = defaultdict(list)
        self.events = []
        self.cmd = (0.0, 0.0)
        self.abort = None
        self.ekf = None
        self.wheel = None
        self.imu_yaw = None
        self.js = None
        self.js_t = None
        self.scans = []
        self.cmd_out = (0.0, 0.0)
        self.cmd_out_nz_since = None
        self.tgt_since = [None, None]
        self.stall_start = [None, None]
        self.motion_hist = []
        self.T_lidar = None
        self.intro_names = {}
        self.T_arena_odom = None
        st = self.load_state()
        if st:
            self.T_arena_odom = tuple(st['T_arena_odom'])
        self.fence_on = True
        self.guard_on = True
        self.clear, self.clear_t = None, None
        self.motion_time = 0.0
        self.test_name = '?'

        sd = qos_profile_sensor_data
        latched = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_ekf, 50)
        self.create_subscription(
            Odometry, '/merlin_base_controller/odom', self.on_wheel, 50
        )
        self.create_subscription(Imu, '/imu/data', self.on_imu, sd)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, sd)
        self.create_subscription(
            TwistStamped, '/merlin_base_controller/cmd_vel_out', self.on_cmd_out, 50
        )
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static, latched)
        self.create_subscription(
            StatisticsNames,
            '/controller_manager/introspection_data/names',
            self.on_intro_names,
            latched,
        )
        self.create_subscription(
            StatisticsValues,
            '/controller_manager/introspection_data/values',
            self.on_intro_values,
            sd,
        )
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 20)
        self.create_timer(0.02, self.tick)

    # ---------- state ----------
    def now(self):
        return time.monotonic() - self.t0

    def load_state(self):
        try:
            with open(STATE) as f:
                return json.load(f)
        except Exception:
            return None

    def save_state(self):
        with open(STATE, 'w') as f:
            json.dump({'T_arena_odom': list(self.T_arena_odom)}, f)

    def event(self, label):
        self.events.append((self.now(), label))
        print(f'[{self.now():7.2f}] {label}', flush=True)

    # ---------- callbacks ----------
    def on_ekf(self, m):
        p = m.pose.pose
        self.ekf = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.rec['ekf'].append(
            (self.now(), *self.ekf, m.twist.twist.linear.x, m.twist.twist.angular.z)
        )

    def on_wheel(self, m):
        p = m.pose.pose
        self.wheel = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.rec['wheel'].append(
            (self.now(), *self.wheel, m.twist.twist.linear.x, m.twist.twist.angular.z)
        )

    def on_imu(self, m):
        self.imu_yaw = yaw_of(m.orientation)
        self.rec['imu'].append(
            (self.now(), self.imu_yaw, m.angular_velocity.z, m.linear_acceleration.x)
        )

    def on_js(self, m):
        try:
            il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
        except ValueError:
            return
        t = self.now()
        self.js = (m.position[il], m.position[ir])
        self.js_t = t
        self.motion_hist.append((t, *self.js))
        if len(self.motion_hist) > 200:
            self.motion_hist = self.motion_hist[-200:]
        self.rec['js'].append(
            (t, m.position[il], m.position[ir], m.velocity[il], m.velocity[ir])
        )

    def on_scan(self, m):
        r = np.array(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.15) & (r < 8.0)
        pts = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
        self.scans.append((self.now(), pts))
        if self.T_lidar is not None:
            self.update_clearance(tf_pts(pts, self.T_lidar))
        if len(self.scans) > 20:
            self.scans = self.scans[-20:]
        self.rec['scan_t'].append((self.now(),))

    def update_clearance(self, pb):
        """Free distance from the footprint ahead, behind and all round; the body hides most of the rear."""
        d = np.hypot(pb[:, 0], pb[:, 1])
        pb, d = (
            pb[d > FOOTPRINT_R],
            d[d > FOOTPRINT_R],
        )  # anything closer is the robot itself
        cone = np.abs(np.arctan2(pb[:, 1], np.abs(pb[:, 0]))) < math.radians(35)
        lane = (np.abs(pb[:, 1]) < FOOTPRINT_R + 0.05) | cone
        gap = d - FOOTPRINT_R
        ahead = gap[(pb[:, 0] > 0) & lane]
        behind = gap[(pb[:, 0] < 0) & lane]
        c = (
            float(ahead.min()) if len(ahead) else 9.0,
            float(behind.min()) if len(behind) else 9.0,
            float(gap.min()) if len(gap) else 9.0,
        )
        self.clear, self.clear_t = c, self.now()
        self.rec['clear'].append((self.clear_t, *c))

    def on_cmd_out(self, m):
        v, w = m.twist.linear.x, m.twist.angular.z
        self.cmd_out = (v, w)
        t = self.now()
        if abs(v) > 1e-3 or abs(w) > 1e-3:
            if self.cmd_out_nz_since is None:
                self.cmd_out_nz_since = t
        else:
            self.cmd_out_nz_since = None
        self.rec['cmd_out'].append((t, v, w))

    def on_tf_static(self, m):
        for tr in m.transforms:
            if tr.child_frame_id == 'lidar_link':
                self.lidar_parent = tr.header.frame_id
                self.T_lidar = (
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    yaw_of(tr.transform.rotation),
                )

    def on_intro_names(self, m):
        self.intro_names[m.names_version] = list(m.names)

    def on_intro_values(self, m):
        names = self.intro_names.get(m.names_version)
        if not names:
            return
        if not hasattr(self, '_intro_idx') or self._intro_ver != m.names_version:
            full = ['Merlin.' + n for n in INTRO]
            try:
                self._intro_idx = [names.index(n) for n in full]
                self._intro_ver = m.names_version
            except ValueError:
                return
        vals = [m.values[i] for i in self._intro_idx]
        t = self.now()
        for k, tg in ((0, vals[0]), (1, vals[4])):
            if abs(tg) > 60:
                if self.tgt_since[k] is None:
                    self.tgt_since[k] = t
            else:
                self.tgt_since[k] = None
        self.rec['intro'].append((t, *vals))

    def on_diag(self, m):
        for s in m.status:
            self.rec['diag'].append(
                (
                    self.now(),
                    s.name,
                    int.from_bytes(s.level, 'little')
                    if isinstance(s.level, bytes)
                    else int(s.level),
                    s.message,
                    {kv.key: kv.value for kv in s.values},
                )
            )

    # ---------- safety ----------
    def tick(self):
        v, w = self.cmd
        t = self.now()
        if self.abort is None:
            # startup is wait_ready's job; only trip once joint_states has been seen
            if self.js_t is not None and t - self.js_t > 0.3:
                self.trip('STALE joint_states')
            if self.fence_on and self.T_arena_odom and self.ekf:
                x, y, _ = compose(self.T_arena_odom, self.ekf)
                if not (FENCE[0] <= x <= FENCE[1] and FENCE[2] <= y <= FENCE[3]):
                    self.trip(f'GEOFENCE arena pose ({x:.3f}, {y:.3f})')
            if self.guard_on and (abs(v) > 0.01 or abs(w) > 0.05):
                self.motion_time += 0.02
                if self.clear is None or t - self.clear_t > 0.5:
                    self.trip('OBSTACLE GUARD: no fresh scan')
                else:
                    ahead, behind, around = self.clear
                    # stop margin grows with speed: 10 Hz scans + 0.8 m/s^2 braking
                    need = 0.08 + 0.15 * abs(v) + v * v / 1.6
                    if v > 0.01 and ahead < need:
                        self.trip(f'OBSTACLE {ahead:.2f} m ahead of the footprint')
                    elif v < -0.01 and behind < need:
                        self.trip(f'OBSTACLE {behind:.2f} m behind the footprint')
                    elif abs(v) <= 0.01 and around < 0.04:
                        self.trip(
                            f'OBSTACLE {around:.2f} m from the footprint while turning'
                        )
            old = [h for h in self.motion_hist if t - h[0] >= 0.6]
            if old and self.js:
                o = old[-1]
                still = [abs(self.js[0] - o[1]) < 1e-3, abs(self.js[1] - o[2]) < 1e-3]
                stuck = [
                    self.tgt_since[k] is not None
                    and t - self.tgt_since[k] > 1.0
                    and still[k]
                    for k in (0, 1)
                ]
                for k in (0, 1):
                    if stuck[k] and self.stall_start[k] is None:
                        self.stall_start[k] = t
                        self.event(
                            f'{"LEFT" if k == 0 else "RIGHT"} wheel stalled (target {self.cmd})'
                        )
                    if not stuck[k]:
                        self.stall_start[k] = None
                if all(stuck):
                    self.power_dropout()
                elif any(stuck) and any(
                    self.stall_start[k] is not None and t - self.stall_start[k] > 1.5
                    for k in (0, 1)
                ):
                    self.trip(
                        f'{"LEFT" if stuck[0] else "RIGHT"} wheel not moving under command: '
                        'wiring / encoder fault?'
                    )
        if self.abort is not None:
            v, w = 0.0, 0.0
        self.publish(v, w)
        self.rec['cmd'].append((t, v, w))

    def power_dropout(self):
        it = self.rec['intro'][-1] if self.rec['intro'] else [0] * 9
        ctx = dict(
            wall=time.strftime('%Y-%m-%d %H:%M:%S'),
            test=self.test_name,
            t_run=round(self.now(), 1),
            motion_s=round(self.motion_time, 1),
            cmd=list(self.cmd),
            target_mrps=[round(it[1]), round(it[5])],
            pwm_permille=[round(it[4]), round(it[8])],
        )
        with open(f'{BENCH}/dropouts.jsonl', 'a') as f:
            f.write(json.dumps(ctx) + '\n')
        self.trip(f'POWER DROPOUT? no edges on both wheels under command: {ctx}')

    def trip(self, reason):
        self.abort = reason
        self.cmd = (0.0, 0.0)
        print(f'!!! ABORT: {reason}', flush=True)

    def publish(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.pub.publish(m)

    def check(self):
        if self.abort is not None:
            raise Abort(self.abort)

    def sleep(self, dt):
        end = time.monotonic() + dt
        while time.monotonic() < end:
            self.check()
            time.sleep(0.005)

    def set_pid(self, kp, ki, kd=0.0):
        if not hasattr(self, 'pid_pub'):
            self.pid_pub = self.create_publisher(
                DynamicInterfaceGroupValues, '/kestrel_tuning_controller/commands', 10
            )
            time.sleep(0.5)
        m = DynamicInterfaceGroupValues()
        m.interface_groups = ['kestrel_pid']
        g = [kp * 1000, ki * 1000, kd * 1000]
        m.interface_values = [
            InterfaceValue(
                interface_names=['kp_l', 'ki_l', 'kd_l', 'kp_r', 'ki_r', 'kd_r'],
                values=[float(x) for x in g + g],
            )
        ]
        for _ in range(3):
            self.pid_pub.publish(m)
            time.sleep(0.1)
        self.event(f'pid kp={kp} ki={ki} kd={kd}')

    # ---------- motion primitives (closed loop on the EKF /odom, as Nav2 would be) ----------
    def set(self, v, w):
        self.check()
        self.cmd = (v, w)

    def stop(self, settle=0.0):
        self.cmd = (0.0, 0.0)
        if settle:
            self.sleep(settle)

    def hold(self, v, w, T):
        self.set(v, w)
        self.sleep(T)

    def rotate_to(self, yaw, wmax=0.8, tol=0.004):
        t_end = time.monotonic() + 20
        while time.monotonic() < t_end:
            err = wrap(yaw - self.ekf[2])
            if abs(err) < tol:
                break
            w = math.copysign(min(wmax, max(0.25, 2.5 * abs(err))), err)
            self.set(0.0, w)
            time.sleep(0.02)
        self.stop()

    def rotate_by(self, ang, wmax=0.8):
        """Relative turn of any size (unwrapped), closed loop on the EKF yaw."""
        done, last = 0.0, self.ekf[2]
        while abs(ang - done) > 0.004:
            yaw = self.ekf[2]
            done += wrap(yaw - last)
            last = yaw
            err = ang - done
            w = math.copysign(min(wmax, max(0.25, 2.5 * abs(err))), err)
            self.set(0.0, w)
            time.sleep(0.02)
        self.stop()

    def drive(self, dist, vmax=0.2, heading=None):
        x0, y0, th = self.ekf
        if heading is not None:
            th = heading
        ux, uy = math.cos(th), math.sin(th)
        t_end = time.monotonic() + 15
        while time.monotonic() < t_end:
            x, y, yaw = self.ekf
            s = (x - x0) * ux + (y - y0) * uy
            rem = dist - s
            if abs(rem) < 0.002:
                break
            v = math.copysign(min(vmax, max(0.025, 1.6 * abs(rem))), rem)
            cross = -(x - x0) * uy + (y - y0) * ux
            w = max(
                -0.4, min(0.4, 2.0 * wrap(th - yaw) - math.copysign(1, v) * 3.0 * cross)
            )
            self.set(v, w)
            time.sleep(0.02)
        self.stop()

    def goto_odom(self, gx, gy, vmax=0.2, wmax=0.8):
        x, y, _ = self.ekf
        d = math.hypot(gx - x, gy - y)
        if d < 0.01:
            return
        bearing = math.atan2(gy - y, gx - x)
        self.rotate_to(bearing, wmax)
        self.stop(0.3)
        self.drive(d, vmax, heading=bearing)
        self.stop(0.3)

    def goto_arena(self, ax, ay, ayaw=None, vmax=0.2):
        Tinv = inverse(self.T_arena_odom)
        gx, gy, _ = compose(Tinv, (ax, ay, 0.0))
        self.goto_odom(gx, gy, vmax)
        if ayaw is not None:
            self.rotate_to(wrap(ayaw + Tinv[2]))
            self.stop(0.3)

    # ---------- ground truth ----------
    def wait_ready(self, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (
                self.ekf
                and self.wheel
                and self.js
                and self.T_lidar
                and len(self.scans) >= 3
            ):
                return
            time.sleep(0.05)
        raise RuntimeError(
            f'not ready: ekf={bool(self.ekf)} wheel={bool(self.wheel)} js={bool(self.js)} '
            f'tf={bool(self.T_lidar)} scans={len(self.scans)}'
        )

    def settled_cloud(self, n=4, settle=0.8):
        self.stop(settle)
        t_start = self.now()
        while len([s for s in self.scans if s[0] > t_start]) < n:
            self.sleep(0.02)
        pts = np.vstack([s[1] for s in self.scans if s[0] > t_start][:n])
        return voxel(tf_pts(pts, self.T_lidar), 0.01)

    def make_reference(self):
        cloud = self.settled_cloud(n=8, settle=1.0)
        n, good = normals(cloud)
        np.savez(REF, ref=cloud[good], n=n[good])
        self.T_arena_odom = compose((0, 0, 0), inverse(self.ekf))
        self.save_state()
        self.event(f'reference made: {good.sum()} pts')

    def keyframe(self, label, n=4):
        cloud = self.settled_cloud(n=n)
        ekf, wheel, imu = self.ekf, self.wheel, self.imu_yaw
        R = np.load(REF)
        init = compose(self.T_arena_odom, ekf)
        best = None
        for dth in (0.0, 0.1, -0.1, 0.25, -0.25):
            T, rmse, inl = icp(
                voxel(cloud, 0.02), R['ref'], R['n'], (init[0], init[1], init[2] + dth)
            )
            if best is None or inl - 5 * rmse > best[2] - 5 * best[1]:
                best = (T, rmse, inl)
            if inl > 0.7 and rmse < 0.018:
                break
        T, rmse, inl = best
        dev = rel(init, T)
        # Odometry can't see the caster scrubbing sideways in spins, so allow a 15 cm correction
        # for a good fit; a 12+ mm residual or a low inlier share is a mismatch, not a correction.
        ok = (inl >= 0.83 and rmse <= 0.0115) or (
            inl >= 0.75
            and rmse <= 0.012
            and math.hypot(dev[0], dev[1]) < 0.15
            and abs(dev[2]) < 0.09
        )
        if ok:
            self.T_arena_odom = compose(T, inverse(ekf))
            self.save_state()
        else:
            self.event(
                f'WARNING keyframe {label}: ICP rejected inl={inl:.2f} rmse={rmse:.4f} '
                f'dev=({dev[0]:+.3f},{dev[1]:+.3f},{math.degrees(dev[2]):+.1f}deg); keeping odom prediction'
            )
        self.kf_clouds[f'{len(self.rec_kf):02d}_{label}'] = cloud
        kf = dict(
            t=self.now(),
            label=label,
            truth=list(T),
            ekf=list(ekf),
            wheel=list(wheel),
            imu=imu,
            rmse=rmse,
            inl=inl,
            accepted=ok,
            pred=list(init),
        )
        self.rec_kf.append(kf)
        self.event(
            f'KF {label}: truth ({T[0]:+.4f},{T[1]:+.4f},{math.degrees(T[2]):+.2f}deg) '
            f'rmse {rmse * 1000:.1f}mm inl {inl:.2f}'
        )
        return kf

    rec_kf = []
    kf_clouds = {}

    def dump(self, name, extra=None):
        out = f'{BENCH}/out'
        os.makedirs(out, exist_ok=True)
        stamp = time.strftime('%H%M%S')
        arrays = {}
        for k, v in self.rec.items():
            if k == 'diag':
                continue
            try:
                arrays[k] = np.array(v, dtype=float)
            except Exception:
                pass
        np.savez_compressed(f'{out}/{name}_{stamp}.npz', **arrays)
        if self.kf_clouds:
            np.savez_compressed(f'{out}/{name}_{stamp}_clouds.npz', **self.kf_clouds)
        diag_last = {}
        for t, nm, lvl, msg, kv in self.rec['diag']:
            diag_last[nm] = dict(t=t, level=lvl, msg=msg, values=kv)
        meta = dict(
            name=name,
            events=self.events,
            keyframes=self.rec_kf,
            abort=self.abort,
            diag_last=diag_last,
            diag_levels=sorted(
                {(nm, lvl, msg) for _, nm, lvl, msg, _ in self.rec['diag']}
            ),
            extra=extra or {},
        )
        with open(f'{out}/{name}_{stamp}.json', 'w') as f:
            json.dump(meta, f, indent=1, default=str)
        print(f'saved {out}/{name}_{stamp}', flush=True)


# =================================== tests ===================================


def t_reference(b):
    b.make_reference()
    b.keyframe('check')


def t_smoke(b):
    b.keyframe('start')
    b.hold(0.08, 0.0, 1.0)
    b.stop(0.8)
    b.hold(-0.08, 0.0, 1.0)
    b.stop()
    b.keyframe('end')


def t_static(b, T=60):
    b.keyframe('start')
    b.event('static hold')
    b.stop(float(T))
    b.keyframe('end')


def t_locate(b):
    """Global relocalisation: grid of seeds, keep the best ICP fit; resets the arena transform."""
    b.fence_on = False
    full = voxel(b.settled_cloud(n=6), 0.02)
    cloud = voxel(full, 0.06)
    R = np.load(REF)
    keep = np.unique(np.floor(R['ref'] / 0.04).astype(int), axis=0, return_index=True)[
        1
    ]
    ref_c, n_c = R['ref'][keep], R['n'][keep]
    b.event(f'locate: {len(cloud)} src / {len(ref_c)} ref pts')
    res = []
    for th in np.radians(np.arange(-180, 180, 20)):
        for x in (-0.3, 0.0, 0.3):
            for y in (-0.3, 0.0, 0.3):
                T, rmse, inl = icp(cloud, ref_c, n_c, (x, y, th), iters=12)
                res.append((inl - 5 * rmse, T, rmse, inl))
    res.sort(key=lambda r: -r[0])
    for sc, T, rmse, inl in res[:6]:
        b.event(
            f'cand ({T[0]:+.3f},{T[1]:+.3f},{math.degrees(T[2]):+7.2f}) rmse {rmse * 1000:.1f} inl {inl:.2f}'
        )
    best = None
    for sc, T0, _, _ in res[:6]:
        T, rmse, inl = icp(full, R['ref'], R['n'], T0, iters=60)
        if best is None or inl - 5 * rmse > best[2] - 5 * best[1]:
            best = (T, rmse, inl)
    T, rmse, inl = best
    b.T_arena_odom = compose(T, inverse(b.ekf))
    b.save_state()
    b.event(
        f'located ({T[0]:+.3f},{T[1]:+.3f},{math.degrees(T[2]):+.2f}) rmse {rmse * 1000:.1f} inl {inl:.2f}'
    )


def t_wheelcheck(b, w=0.8, T=1.5):
    b.keyframe('start')
    for s in (1, -1, 1, -1):
        j0 = b.js
        b.event(f'check spin {s * w:+.2f}')
        b.hold(0.0, s * w, T)
        b.stop(1.0)
        b.event(f'  dL {b.js[0] - j0[0]:+.2f} dR {b.js[1] - j0[1]:+.2f} rad')
    b.keyframe('end')


def t_home(b):
    b.fence_on = False
    b.keyframe('before')
    b.goto_arena(0.0, 0.0, 0.0, vmax=0.12)
    b.fence_on = True
    b.keyframe('home')


def t_backoff(b, d=0.15):
    """Recovery: reverse straight back over the ground just driven (the lidar can't see behind)."""
    b.fence_on = False
    b.guard_on = False
    b.drive(-abs(d), 0.08)
    b.stop(0.5)


def t_centre(b):
    b.keyframe('before')
    b.goto_arena(*CENTRE, 0.0, vmax=0.12)
    b.keyframe('centre')


def t_pid(b, kp=1.0, ki=2.0, part='spin', T=2.5):
    """Live gain test without lidar truth: in-place spin steps, or the low-speed set around the centre."""
    b.set_pid(kp, ki)
    b.stop(0.5)
    if part == 'spin':
        for w in (0.3, 0.8, 1.5):
            for s in (1, -1):
                b.event(f'spin step {s * w:+.2f}')
                b.hold(0.0, s * w, T)
                b.event('spin stop')
                b.stop(1.2)
    else:
        for v in (0.03, 0.05):
            for s in (1, -1):
                b.event(f'low lin {s * v:+.3f}')
                b.hold(s * v, 0.0, 0.08 / v)
                b.event('low stop')
                b.stop(1.0)
        for w in (0.25, 0.4):
            for s in (1, -1):
                b.event(f'low ang {s * w:+.2f}')
                b.hold(0.0, s * w, 0.6 / w)
                b.event('low stop')
                b.stop(1.0)


def t_spin_steps(b, ws=(0.3, 0.8, 1.5), T=3.0):
    """In-place spin steps: both wheels run at |w|*sep/2 while the robot stays put."""
    b.keyframe('start')
    for w in ws:
        for s in (1, -1):
            b.event(f'spin step {s * w:+.2f}')
            b.hold(0.0, s * w, T)
            b.event('spin stop')
            b.stop(1.2)
    b.keyframe('end')


def t_lin_steps(b, vs=(0.1, 0.2, 0.3), L=0.9):
    """Open-loop straight steps of length L along +x, from x=-0.78 (reposition is EKF closed loop)."""
    b.keyframe('start')
    for v in vs:
        b.goto_arena(-0.78, 0.0, 0.0, vmax=0.15)
        b.keyframe(f'lin{v}_a')
        b.event(f'lin step {v:+.2f}')
        x0 = b.ekf
        T = L / v
        b.set(v, 0.0)
        t_end = time.monotonic() + T + 1.0
        while time.monotonic() < t_end:
            dx = (b.ekf[0] - x0[0]) * math.cos(x0[2]) + (b.ekf[1] - x0[1]) * math.sin(
                x0[2]
            )
            if dx > L - v * v / 1.6 - 0.01:
                break
            time.sleep(0.01)
        b.event('lin stop')
        b.stop(1.0)
        b.keyframe(f'lin{v}_b')
    b.goto_arena(*CENTRE, 0.0, vmax=0.15)
    b.keyframe('end')


def t_lowspeed(b):
    b.keyframe('start')
    b.goto_arena(CENTRE[0] - 0.15, CENTRE[1], 0.0, vmax=0.12)
    for v in (0.02, 0.03, 0.05):
        b.event(f'low lin {v:+.3f}')
        b.hold(v, 0.0, 0.10 / v)
        b.event('low stop')
        b.stop(1.0)
        b.event(f'low lin {-v:+.3f}')
        b.hold(-v, 0.0, 0.10 / v)
        b.event('low stop')
        b.stop(1.0)
    b.goto_arena(*CENTRE, 0.0, vmax=0.12)
    for w in (0.1, 0.2, 0.3):
        b.event(f'low ang {w:+.2f}')
        b.hold(0.0, w, 0.8 / w)
        b.event('low stop')
        b.stop(1.0)
        b.event(f'low ang {-w:+.2f}')
        b.hold(0.0, -w, 0.8 / w)
        b.event('low stop')
        b.stop(1.0)
    b.keyframe('end')


def t_arc(b, combos=((0.30, 1.2), (0.35, 1.4))):
    """Circle of radius v/w around the arena centre: start at (0, -r) heading +x."""
    b.keyframe('start')
    for v, w in combos:
        r = v / w
        b.goto_arena(CENTRE[0], CENTRE[1] - r, 0.0, vmax=0.12)
        b.keyframe(f'arc{v}_{w}_a')
        b.event(f'arc {v:.2f} {w:.2f}')
        y0 = b.imu_yaw
        turned, last = 0.0, y0
        b.set(v, w)
        t_end = time.monotonic() + 2 * math.pi / w * 1.6
        while time.monotonic() < t_end and turned < 2 * math.pi - 0.25:
            turned += wrap(b.imu_yaw - last)
            last = b.imu_yaw
            b.check()
            time.sleep(0.01)
        b.event('arc stop')
        b.stop(1.0)
        b.keyframe(f'arc{v}_{w}_b')
    b.goto_arena(*CENTRE, 0.0, vmax=0.12)
    b.keyframe('end')


def t_reversal(b, v=0.2, n=3):
    b.keyframe('start')
    b.goto_arena(CENTRE[0] - 0.15, CENTRE[1], 0.0, vmax=0.12)
    b.keyframe('rev_a')
    for i in range(n):
        b.event(f'rev fwd {v:+.2f}')
        b.hold(v, 0.0, 1.3)
        b.event(f'rev back {-v:+.2f}')
        b.hold(-v, 0.0, 1.3)
        b.event('rev stop')
        b.stop(1.0)
    b.keyframe('rev_b')
    for i in range(n):
        b.event(f'rev spin {1.2:+.2f}')
        b.hold(0.0, 1.2, 1.2)
        b.event(f'rev spin {-1.2:+.2f}')
        b.hold(0.0, -1.2, 1.2)
        b.event('rev stop')
        b.stop(1.0)
    b.keyframe('rev_c')
    b.goto_arena(*CENTRE, 0.0, vmax=0.12)
    b.keyframe('end')


def t_rot(b, turns=2, w=1.0):
    """Multi-turn spins for the angular scale: truth from IMU abs yaw (unwrapped) + ICP at the ends."""
    b.goto_arena(*CENTRE, 0.0, vmax=0.12)
    b.keyframe('start')
    for s in (1, -1):
        b.event(f'rot {s * turns} turns')
        b.rotate_by(s * turns * 2 * math.pi, wmax=w)
        b.stop()
        b.keyframe(f'rot{s * turns}')
    b.keyframe('end')


def t_umb(b, side=0.6, runs=5, v=0.2, w=0.8, dirs='ccw,cw', corner_kf_runs=1):
    """Scaled UMBmark: square of `side` centred on CENTRE, followed on EKF odometry.

    Start corner (-s/2,-s/2) heading +x for CCW, (-s/2,+s/2) heading +x for CW. The first
    `corner_kf_runs` runs per direction also take a keyframe at each corner (IMU vs ICP yaw)."""
    h = side / 2
    cx, cy = CENTRE
    b.keyframe('start')
    for d in dirs.split(','):
        for i in range(runs):
            sy = -h if d == 'ccw' else h
            b.goto_arena(cx - h, cy + sy, 0.0, vmax=0.15)
            b.keyframe(f'umb_{d}{i}_start')
            b.event(f'umb {d} run {i}')
            turn = math.pi / 2 if d == 'ccw' else -math.pi / 2
            x0, y0, th0 = b.ekf
            for k in range(4):
                hd = th0 + k * turn
                b.drive(side, v, heading=hd)
                b.stop(0.4)
                if i < corner_kf_runs and k < 3:
                    b.keyframe(f'umb_{d}{i}_c{k + 1}')
                b.rotate_to(wrap(hd + turn), w)
                b.stop(0.4)
            b.event(f'umb {d} run {i} done')
            b.keyframe(f'umb_{d}{i}_end')
    b.goto_arena(*CENTRE, 0.0, vmax=0.15)
    b.keyframe('end')


TESTS = dict(
    locate=t_locate,
    wheelcheck=t_wheelcheck,
    reference=t_reference,
    smoke=t_smoke,
    static=t_static,
    home=t_home,
    spin=t_spin_steps,
    lin=t_lin_steps,
    low=t_lowspeed,
    arc=t_arc,
    rev=t_reversal,
    rot=t_rot,
    umb=t_umb,
    backoff=t_backoff,
    centre=t_centre,
    pid=t_pid,
)


def main():
    name = sys.argv[1]
    kwargs = {}
    for a in sys.argv[2:]:
        k, v = a.split('=')
        try:
            kwargs[k] = json.loads(v)
        except Exception:
            kwargs[k] = v
        if isinstance(kwargs[k], list):
            kwargs[k] = tuple(tuple(x) if isinstance(x, list) else x for x in kwargs[k])
    rclpy.init()
    b = Bench()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(b)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def on_sig(signum, frame):
        b.trip(f'signal {signum}')
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sig)
    signal.signal(signal.SIGINT, on_sig)
    try:
        b.wait_ready()
        if name != 'reference' and not os.path.exists(REF):
            raise RuntimeError('no reference; run `reference` first')
        if b.T_arena_odom is None and name != 'reference':
            raise RuntimeError('no arena state')
        pubs = b.count_publishers('/cmd_vel')
        if pubs > 1:
            raise RuntimeError(f'{pubs} publishers on /cmd_vel; stop teleop first')
        b.test_name = name
        b.event(
            f'start {name} {kwargs} lidar {getattr(b, "lidar_parent", "?")} {b.T_lidar}'
        )
        TESTS[name](b, **kwargs)
        b.event(f'done {name}')
    except (Abort, KeyboardInterrupt) as e:
        b.event(f'ABORTED: {b.abort or e}')
    except Exception as e:
        b.trip(f'exception {e!r}')
        import traceback

        traceback.print_exc()
    finally:
        b.cmd = (0.0, 0.0)
        b.abort = b.abort or None
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            b.publish(0.0, 0.0)
            time.sleep(0.02)
        ex.shutdown()  # stop callbacks before dump iterates self.rec
        b.dump(name, extra=kwargs)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
