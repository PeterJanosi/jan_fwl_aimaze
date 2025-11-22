#!/usr/bin/env python3
"""Simple ROS2 node that publishes fake /scan LaserScan messages for testing.

Usage:
  python3 scripts/fake_scan_publisher.py --rate 5 --ranges 360 --range_max 3.5

This is a convenience helper for local smoke tests of the RL environment.
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self, topic: str = '/scan', rate_hz: float = 5.0, ranges_count: int = 360, range_max: float = 3.5):
        super().__init__('fake_scan_publisher')
        self.pub = self.create_publisher(LaserScan, topic, 10)
        self.timer_period = 1.0 / float(rate_hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.ranges_count = int(ranges_count)
        self.angle_increment = (self.angle_max - self.angle_min) / max(1, (self.ranges_count - 1))
        self.range_max = float(range_max)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max)
        msg.angle_increment = float(self.angle_increment)
        msg.time_increment = 0.0
        msg.scan_time = float(self.timer_period)
        msg.range_min = 0.05
        msg.range_max = float(self.range_max)
        # default: clear environment (all max range)
        msg.ranges = [self.range_max for _ in range(self.ranges_count)]
        msg.intensities = []
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description='Fake /scan publisher for RL smoke tests')
    parser.add_argument('--topic', default='/scan', help='Topic to publish LaserScan on')
    parser.add_argument('--rate', type=float, default=5.0, help='Publish rate in Hz')
    parser.add_argument('--ranges', type=int, default=360, help='Number of scan ranges')
    parser.add_argument('--range_max', type=float, default=3.5, help='Maximum range value')
    args = parser.parse_args()

    rclpy.init()
    node = FakeScan(topic=args.topic, rate_hz=args.rate, ranges_count=args.ranges, range_max=args.range_max)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
