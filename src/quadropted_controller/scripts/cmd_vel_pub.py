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

        # 加减速平滑斜率 (Slew-Rate Limiter)
        self.declare_parameter('enable_slew_rate', True)
        self.declare_parameter('accel_limit_x', 0.60)     # m/s^2 对地等效加速度
        self.declare_parameter('accel_limit_y', 0.60)     # m/s^2 对地等效横向加速度
        self.declare_parameter('accel_limit_yaw', 1.50)   # rad/s^2 角加速度
        self.declare_parameter('cmd_timeout', 0.50)       # 指令超时自动减速回零

        # 内部状态
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_timer_time = self.get_clock().now()

        # 50Hz 平滑控制循环
        self.timer = self.create_timer(0.02, self.control_loop)

    def robot_velocity_callback(self, msg: Twist):
        self.target_x = msg.linear.x
        self.target_y = msg.linear.y
        self.target_z = msg.linear.z
        self.target_yaw = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

        # 如果没有开启平滑，直接调用即刻发布
        if not self.get_parameter('enable_slew_rate').get_parameter_value().bool_value:
            self.publish_cmd(self.target_x, self.target_y, self.target_z, self.target_yaw)

    def control_loop(self):
        if not self.get_parameter('enable_slew_rate').get_parameter_value().bool_value:
            return

        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        # 超时保护：超过 cmd_timeout 未收到 cmd_vel，目标速度归零
        cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        if (now - self.last_cmd_time).nanoseconds / 1e9 > cmd_timeout:
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = 0.0
            self.target_yaw = 0.0

        accel_x = self.get_parameter('accel_limit_x').get_parameter_value().double_value
        accel_y = self.get_parameter('accel_limit_y').get_parameter_value().double_value
        accel_yaw = self.get_parameter('accel_limit_yaw').get_parameter_value().double_value

        self.current_x += self.clamp(self.target_x - self.current_x, -accel_x * dt, accel_x * dt)
        self.current_y += self.clamp(self.target_y - self.current_y, -accel_y * dt, accel_y * dt)
        self.current_z = self.target_z
        self.current_yaw += self.clamp(self.target_yaw - self.current_yaw, -accel_yaw * dt, accel_yaw * dt)

        # 仅在非静止或刚停下时发布，避免无用开销
        has_motion = (abs(self.current_x) > 1e-4 or abs(self.current_y) > 1e-4 or abs(self.current_yaw) > 1e-4 or
                      abs(self.target_x) > 1e-4 or abs(self.target_y) > 1e-4 or abs(self.target_yaw) > 1e-4)

        if has_motion or self.motion_start_time is not None:
            self.publish_cmd(self.current_x, self.current_y, self.current_z, self.current_yaw)

    def clamp(self, val, min_v, max_v):
        return max(min_v, min(val, max_v))

    def publish_cmd(self, vx, vy, vz, vyaw):
        has_velocity = (abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(vz) > 1e-4 or abs(vyaw) > 1e-4)
        current_time = self.get_clock().now()

        # 秒表与状态管理
        if has_velocity and self.motion_start_time is None:
            self.motion_start_time = current_time
            if self.verbose:
                self.get_logger().info(f"Motion started at time: {current_time.to_msg()}")

        if not has_velocity and self.motion_start_time is not None:
            elapsed = current_time - self.motion_start_time
            if self.verbose:
                self.get_logger().info(f"Motion stopped at time: {current_time.to_msg()}")
                self.get_logger().info(f"Elapsed motion time: {elapsed.nanoseconds / 1e9:.3f} seconds")
            self.motion_start_time = None

        scale_x = self.get_parameter('scale_x').get_parameter_value().double_value
        scale_y = self.get_parameter('scale_y').get_parameter_value().double_value
        scale_yaw = self.get_parameter('scale_yaw').get_parameter_value().double_value
        max_x = self.get_parameter('max_x').get_parameter_value().double_value
        max_y = self.get_parameter('max_y').get_parameter_value().double_value
        max_yaw = self.get_parameter('max_yaw').get_parameter_value().double_value

        new_msg = RobotVelocity()
        new_msg.robot_id = 1

        # 线性缩放与限幅
        new_msg.cmd_vel.linear.x = self.scale_and_limit(vx, scale_x, -max_x, max_x)
        new_msg.cmd_vel.linear.y = self.scale_and_limit(vy, scale_y, -max_y, max_y)
        new_msg.cmd_vel.linear.z = vz

        new_msg.cmd_vel.angular.x = 0.0
        new_msg.cmd_vel.angular.y = 0.0
        new_msg.cmd_vel.angular.z = self.scale_and_limit(vyaw, scale_yaw, -max_yaw, max_yaw)

        self.publisher_.publish(new_msg)
        if self.verbose:
            self.get_logger().info(
                f"Published RobotVelocity: robot_id={new_msg.robot_id}, "
                f"linear=({new_msg.cmd_vel.linear.x}, {new_msg.cmd_vel.linear.y}, {new_msg.cmd_vel.linear.z}), "
                f"angular=({new_msg.cmd_vel.angular.z})"
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