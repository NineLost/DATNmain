#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class DualLidarMerger(Node):
    def __init__(self):
        super().__init__('dual_lidar_merger')

        # ===== SUBSCRIBERS =====
        self.sub_front = self.create_subscription(
            LaserScan, '/scan_front', self.cb_front, 10)

        self.sub_rear = self.create_subscription(
            LaserScan, '/scan_rear', self.cb_rear, 10)

        # ===== PUBLISHER =====
        self.pub = self.create_publisher(
            LaserScan, '/scan_merged', 10)

        self.front = None
        self.rear = None

        # ===== STATIC TF: base_link → laser_merged =====
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.get_logger().info('Dual Lidar Merger READY')

    # ==================================================
    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_merged'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    # ==================================================
    def cb_front(self, msg: LaserScan):
        self.front = msg
        self.try_publish()

    def cb_rear(self, msg: LaserScan):
        self.rear = msg
        self.try_publish()

    # ==================================================
    def try_publish(self):
        if self.front is None or self.rear is None:
            return

        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = 'laser_merged'

        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = self.front.angle_increment

        merged.range_min = min(self.front.range_min, self.rear.range_min)
        merged.range_max = max(self.front.range_max, self.rear.range_max)

        # ===== MERGE RANGES =====
        merged.ranges = list(self.front.ranges) + list(self.rear.ranges)

        self.pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()

