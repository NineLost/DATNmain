#!/usr/bin/env python3 

import math 

import struct 

import serial 

import rclpy 

from rclpy.node import Node 

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

from sensor_msgs.msg import Imu 

 

G = 9.80665  # m/s^2 

 

def euler_to_quat(roll, pitch, yaw): 

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5) 

    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5) 

    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5) 

    w = cr * cp * cy + sr * sp * sy 

    x = sr * cp * cy - cr * sp * sy 

    y = cr * sp * cy + sr * cp * sy 

    z = cr * cp * sy - sr * sp * cy 

    return [w, x, y, z] 

 

class HWT911(Node): 

    def __init__(self): 

        super().__init__('hwt911') 

        # params 

        port = self.declare_parameter('port', '/dev/imu').value 

        baud = int(self.declare_parameter('baud', 57600).value) 

        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value 

 

        # publisher (QoS sensor) 

        qos = QoSProfile(depth=50) 

        qos.reliability = ReliabilityPolicy.RELIABLE 

        qos.history = HistoryPolicy.KEEP_LAST 

        self.pub = self.create_publisher(Imu, 'imu/data', qos) 

 

        # serial 

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02) 

        self.get_logger().info(f"Opened {port} @ {baud}") 

        self.buf = bytearray() 

 

        # states 

        self.acc = None 

        self.gyro = None 

        self.quat = None 

 

        # timer 

        self.create_timer(0.01, self.spin_serial) 

 

    def spin_serial(self): 

        try: 

            n = self.ser.in_waiting 

            if n: 

                self.buf += self.ser.read(n) 

                self.parse() 

        except Exception as e: 

            self.get_logger().error(f"serial error: {e}") 

 

    def parse(self): 

        # WIT protocol: 0x55, pid, 8 bytes payload, checksum 

        while len(self.buf) >= 11: 

            # align to header 0x55 

            if self.buf[0] != 0x55: 

                self.buf.pop(0) 

                continue 

 

            pkt = self.buf[:11] 

            if ((sum(pkt[:10]) & 0xFF) != pkt[10]): 

                # sai checksum -> bỏ 1 byte và tìm lại 

                self.buf.pop(0) 

                continue 

 

            pid = pkt[1] 

            pay = pkt[2:10] 

 

            if pid == 0x51:  # Acc (g) -> m/s^2  (scale ±16g: 16/32768) 

                ax, ay, az, _ = struct.unpack('<hhhh', pay) 

                self.acc = ( 

                    float(ax) / 32768.0 * 16.0 * G, 

                    float(ay) / 32768.0 * 16.0 * G, 

                    float(az) / 32768.0 * 16.0 * G, 

                ) 

 

            elif pid == 0x52:  # Gyro (deg/s) -> rad/s (scale ±2000 dps) 

                gx, gy, gz, _ = struct.unpack('<hhhh', pay) 

                d2r = math.pi / 180.0 

                self.gyro = ( 

                    float(gx) / 32768.0 * 2000.0 * d2r, 

                    float(gy) / 32768.0 * 2000.0 * d2r, 

                    float(gz) / 32768.0 * 2000.0 * d2r, 

                ) 

 

            elif pid == 0x53:  # Angle (Euler degrees) -> quaternion 

                r, p, y, _ = struct.unpack('<hhhh', pay) 

                deg2rad = math.pi / 180.0 

                roll  = (float(r) / 32768.0) * 180.0 * deg2rad 

                pitch = (float(p) / 32768.0) * 180.0 * deg2rad 

                yaw   = (float(y) / 32768.0) * 180.0 * deg2rad 

                self.quat = euler_to_quat(roll, pitch, yaw) 

 

            elif pid == 0x59:  # Quaternion (1/32768) 

                q0, q1, q2, q3 = struct.unpack('<hhhh', pay) 

                self.quat = [ 

                    float(q0) / 32768.0, 

                    float(q1) / 32768.0, 

                    float(q2) / 32768.0, 

                    float(q3) / 32768.0, 

                ] 

 

            # publish mỗi khi có acc & gyro (quat có/không vẫn phát, nếu chưa thì quat = identity) 

            if (self.acc is not None) and (self.gyro is not None): 

                if self.quat is None: 

                    self.quat = [1.0, 0.0, 0.0, 0.0] 

                self.publish_imu() 

 

            # bỏ gói đã xử lý 

            del self.buf[:11] 

 

    def publish_imu(self): 

        msg = Imu() 

        msg.header.stamp = self.get_clock().now().to_msg() 

        msg.header.frame_id = self.frame_id 

 

        # orientation 

        q = self.quat if self.quat is not None else [1.0, 0.0, 0.0, 0.0] 

        msg.orientation.w = float(q[0]) 

        msg.orientation.x = float(q[1]) 

        msg.orientation.y = float(q[2]) 

        msg.orientation.z = float(q[3]) 

 

        # covariance: 9 floats 

        msg.orientation_covariance = [ 

            0.02, 0.0, 0.0, 

            0.0,  0.02, 0.0, 

            0.0,  0.0,  0.04 

        ] 

 

        # angular velocity (rad/s) 

        gx, gy, gz = self.gyro 

        msg.angular_velocity.x = float(gx) 

        msg.angular_velocity.y = float(gy) 

        msg.angular_velocity.z = float(gz) 

        msg.angular_velocity_covariance = [ 

            0.02, 0.0, 0.0, 

            0.0,  0.02, 0.0, 

            0.0,  0.0,  0.02 

        ] 

 

        # linear acceleration (m/s^2) 

        ax, ay, az = self.acc 

        msg.linear_acceleration.x = float(ax) 

        msg.linear_acceleration.y = float(ay) 

        msg.linear_acceleration.z = float(az) 

        msg.linear_acceleration_covariance = [ 

            0.04, 0.0, 0.0, 

            0.0,  0.04, 0.0, 

            0.0,  0.0,  0.04 

        ] 

 

        self.pub.publish(msg) 

 

def main(): 

    rclpy.init() 

    n = HWT911() 

    rclpy.spin(n) 

    n.destroy_node() 

    rclpy.shutdown() 

 

if __name__ == '__main__': 

    main() 
