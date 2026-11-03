#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotVelocity
import math

class RobotVelocityHandler(Node):
    def __init__(self):
        super().__init__('robot_velocity_handler')

        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        if self.verbose:
            self.get_logger().info(f"Verbose mode: {self.verbose}")

        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel',
            self.robot_velocity_callback, 
            10)

        self.publisher_ = self.create_publisher(RobotVelocity, 'robot_velocity', 10)
        if self.verbose:
            self.get_logger().info("Node started: RobotVelocityHandler")

        # 秒表变量
        self.motion_start_time = None

        # 速度缩放与限幅参数（经 Gazebo Harmonic 动力学实测标定，误差 <= 5%）
        # 标定依据：对地线速度响应比 k_x ≈ 12.88，k_y ≈ 9.60
        # scale_x = 1 / 12.88 ≈ 0.0776；scale_y = 1 / 9.60 ≈ 0.104
        # max_x = 0.045：在维持机身俯仰与足端抓地力极限下最大安全步幅输出（地速约 0.48~0.52 m/s）
        # max_y = 0.025：横向最大安全步幅输出（横向地速约 0.20~0.24 m/s）
        self.declare_parameter('scale_x', 0.0776)
        self.declare_parameter('scale_y', 0.104)
        self.declare_parameter('scale_yaw', 0.40)
        self.declare_parameter('max_x', 0.045)
        self.declare_parameter('max_y', 0.025)
        self.declare_parameter('max_yaw', 0.50)

    def robot_velocity_callback(self, msg: Twist):
        # 确定是否存在非零速度
        has_velocity = (msg.linear.x != 0 or msg.linear.y != 0 or 
                        msg.linear.z != 0 or 
                        msg.angular.x != 0 or msg.angular.y != 0 or msg.angular.z != 0)

        current_time = self.get_clock().now()

        # 如果速度不为零且秒表未运行，则启动秒表。
        if has_velocity and self.motion_start_time is None:
            self.motion_start_time = current_time
            if self.verbose:
                self.get_logger().info(f"Motion started at time: {current_time.to_msg()}")

        # 如果没有速度（机器人已停止）且秒表正在运行，则停止秒表。
        if not has_velocity and self.motion_start_time is not None:
            elapsed = current_time - self.motion_start_time
            if self.verbose:
                self.get_logger().info(f"Motion stopped at time: {current_time.to_msg()}")
                self.get_logger().info(f"Elapsed motion time: {elapsed.nanoseconds / 1e9:.3f} seconds")
            self.motion_start_time = None
        if self.verbose:
            self.get_logger().info(
                f"Received Twist: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
                f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})"
            )

        scale_x = self.get_parameter('scale_x').get_parameter_value().double_value
        scale_y = self.get_parameter('scale_y').get_parameter_value().double_value
        scale_yaw = self.get_parameter('scale_yaw').get_parameter_value().double_value
        max_x = self.get_parameter('max_x').get_parameter_value().double_value
        max_y = self.get_parameter('max_y').get_parameter_value().double_value
        max_yaw = self.get_parameter('max_yaw').get_parameter_value().double_value

        new_msg = RobotVelocity()
        new_msg.robot_id = 1

        # 线性缩放与限幅：直接响应 teleop w/x 加减速，解决指数饱和导致速度固定锁死的 Bug
        new_msg.cmd_vel.linear.x = self.scale_and_limit(msg.linear.x, scale_x, -max_x, max_x)
        new_msg.cmd_vel.linear.y = self.scale_and_limit(msg.linear.y, scale_y, -max_y, max_y)
        new_msg.cmd_vel.linear.z = msg.linear.z

        new_msg.cmd_vel.angular.x = msg.angular.x
        new_msg.cmd_vel.angular.y = msg.angular.y
        new_msg.cmd_vel.angular.z = self.scale_and_limit(msg.angular.z, scale_yaw, -max_yaw, max_yaw)

        self.publisher_.publish(new_msg)
        if self.verbose:
            self.get_logger().info(
                f"Published RobotVelocity: robot_id={new_msg.robot_id}, "
                f"linear=({new_msg.cmd_vel.linear.x}, {new_msg.cmd_vel.linear.y}, {new_msg.cmd_vel.linear.z}), "
                f"angular=({new_msg.cmd_vel.angular.x}, {new_msg.cmd_vel.angular.y}, {new_msg.cmd_vel.angular.z})"
            )
        
    def scale_and_limit(self, value, scale_factor, min_limit, max_limit):
        scaled = value * scale_factor
        if scaled > max_limit:
            return max_limit
        elif scaled < min_limit:
            return min_limit
        return scaled

    def multiply_and_limit(self, value, scale_factor, min_limit, max_limit):
        # 兼容旧接口别名
        return self.scale_and_limit(value, scale_factor, min_limit, max_limit)

    def limit_value(self, value, min_limit, max_limit):
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

    def limit(self, value, min_limit, max_limit):
        if value > max_limit:
            return max_limit
        elif value < min_limit:
            return min_limit
        else:
            return value

def main(args=None):
    rclpy.init(args=args)
    node = RobotVelocityHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()