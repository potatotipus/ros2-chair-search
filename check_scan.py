#!/usr/bin/env python3
"""Read-only ROS 2 scan and TF diagnostic. Does not command motion."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def summarize(scan):
    values = list(scan.ranges)
    finite = [r for r in values if math.isfinite(r)]
    valid = sorted(r for r in finite if scan.range_min <= r <= scan.range_max and r > 0)
    return {
        'count': len(values), 'valid': len(valid),
        'inf': sum(math.isinf(r) for r in values),
        'nan': sum(math.isnan(r) for r in values),
        'zero_or_negative': sum(r <= 0 for r in finite),
        'outside_limits': sum(r < scan.range_min or r > scan.range_max for r in finite),
        'min': valid[0] if valid else None,
        'median': valid[len(valid) // 2] if valid else None,
        'max': valid[-1] if valid else None,
    }


def show_tf(buffer, parent, child):
    try:
        tf = buffer.lookup_transform(parent, child, Time())
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        print(f'TF {parent} <- {child}: '
              f'xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
              f'quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}), '
              f'yaw_deg={math.degrees(yaw):.1f}')
    except Exception as error:
        print(f'TF {parent} <- {child}: ERROR: {error}')


def main():
    rclpy.init()
    node = Node('scan_diagnostic')
    samples = []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscription = node.create_subscription(
        LaserScan, '/scan', lambda message: samples.append(message), qos_profile_sensor_data)
    deadline = time.monotonic() + 8.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(samples) > 20:
                samples.pop(0)
        if not samples:
            print('NO SCAN RECEIVED in 8 wall-clock seconds.')
            return
        scan = samples[-1]
        print(f'Recent scan messages: {len(samples)} (last 20 retained)')
        print(f'Frame: {scan.header.frame_id}')
        print(f'Stamp: {scan.header.stamp.sec}.{scan.header.stamp.nanosec:09d}')
        print(f'Simulation now: {node.get_clock().now().nanoseconds / 1e9:.3f}')
        print(f'Range limits (m): {scan.range_min:.3f} .. {scan.range_max:.3f}')
        print(f'Angles (rad): {scan.angle_min:.3f} .. {scan.angle_max:.3f}')
        print('Range statistics:', summarize(scan))
        print('First 12 ranges:', list(scan.ranges[:12]))
        show_tf(buffer, 'map', 'odom')
        show_tf(buffer, 'map', 'base_link')
        show_tf(buffer, 'base_link', scan.header.frame_id)
    finally:
        node.destroy_subscription(subscription)
        listener.unregister()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
