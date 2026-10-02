#!/usr/bin/env python3
"""Publish a /cmd_vel step sequence at 20 Hz, then stop:  python3 seq.py v,w,secs [v,w,secs ...]

  python3 seq.py 0,0.8,4 0,0,2 0,-0.8,4      # spin step up, rest, spin the other way
No fence or lidar stop here. Ctrl-C publishes zero; if this process dies the base still
brakes within diff_drive's cmd_vel_timeout (0.5 s).
"""

import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.signals import SignalHandlerOptions


def main():
    steps = [tuple(map(float, s.split(','))) for s in sys.argv[1:]]
    if not steps or any(len(s) != 3 for s in steps):
        sys.exit(__doc__)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('cmd_vel_seq')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def send(v, w):
        m = (
            TwistStamped()
        )  # zero stamp: diff_drive stamps it on arrival, so clock skew can't age it out
        m.twist.linear.x, m.twist.angular.z = v, w
        pub.publish(m)

    while pub.get_subscription_count() == 0:
        time.sleep(0.1)
    try:
        for v, w, secs in steps + [(0.0, 0.0, 1.0)]:
            print(f'v={v:+.3f} m/s  w={w:+.3f} rad/s  for {secs:g} s', flush=True)
            end = time.monotonic() + secs
            while time.monotonic() < end:
                send(v, w)
                time.sleep(0.05)
    finally:
        for _ in range(5):
            send(0.0, 0.0)
            time.sleep(0.05)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
