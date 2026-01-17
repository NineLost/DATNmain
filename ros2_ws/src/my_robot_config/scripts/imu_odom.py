#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuOdom(Node):
    """
    IMU → publish yaw của robot
    - Yaw thuộc về base_link
    - KHÔNG publish TF
    - KHÔNG ảnh hưởng odom
    """

    def __init__(self):
        super().__init__('imu_odom')

        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.cb_imu,
            50
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom_imu',
            20
        )

        self.get_logger().info("imu_odom READY (yaw only, base_link frame)")

    def cb_imu(self, msg: Imu):
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "base_link"
        odom.child_frame_id = "base_link"

        # chỉ mang yaw, không position
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)

        odom.twist.twist.angular.z = msg.angular_velocity.z

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

