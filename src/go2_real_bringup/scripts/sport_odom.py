#!/usr/bin/env python3
# sport_odom.py：sport 状态 → nav_msgs/Odometry（EKF 的 odom0 输入）
#
# 订阅官方 lf/sportmodestate（绝对名），输出相对名 odom（随 namespace 到 /robot1/odom）。
# 只发话题，不发 TF：odom→base_link 由 EKF 发（TF 单发布者规则，见 AGENTS.md）。
# 字段以官方消息定义为准：
#   unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg
#   position[3] / velocity[3] / imu_state.rpy[2](yaw) / yaw_speed
#
# 待实测：sport 的 position/yaw 坐标系约定（若与 ROS ENU 不一致，这里加符号/旋转修正）；
# 协方差是拍脑袋初值，看 EKF 收敛情况再调；lf 版频率若太低（<10Hz），换高频 sportmodestate。

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
from unitree_go.msg import SportModeState


def cov6(diag):
    c = [0.0] * 36
    for i, v in enumerate(diag):
        c[i * 6 + i] = v
    return c


class SportOdom(Node):
    def __init__(self):
        super().__init__('sport_odom')
        self.declare_parameter('sport_topic', '/lf/sportmodestate')  # 绝对名
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('zero_on_start', True)  # 起点对零（对标官方例程 GetInitState）
        # yaw_speed 死区：机载陀螺静止零偏约 0.005~0.02 rad/s；EKF 只积分 yaw_speed
        # （yaw pose 未融合），零偏会直接积成肉眼可见的自转。小于死区的按 0 处理。
        # 建图转弯角速度约 0.3rad/s，死区 0.01 不影响正常运动。
        self.declare_parameter('yaw_speed_deadzone', 0.01)
        self.sport_topic = self.get_parameter('sport_topic').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.zero_on_start = self.get_parameter('zero_on_start').value
        self.yaw_deadzone = self.get_parameter('yaw_speed_deadzone').value

        # 宇树 DDS 广播使用 Best Effort QoS，必须显式匹配
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(SportModeState, self.sport_topic,
                                 self.sport_callback, sensor_qos)
        # 兼容不同固件版本的话题命名
        if self.sport_topic != '/sportmodestate':
            self.create_subscription(SportModeState, '/sportmodestate',
                                     self.sport_callback, sensor_qos)
        self._origin = None  # (x0, y0, yaw0)

    def sport_callback(self, msg: SportModeState):
        x, y = float(msg.position[0]), float(msg.position[1])
        yaw = float(msg.imu_state.rpy[2])
        if self.zero_on_start:
            if self._origin is None:
                self._origin = (x, y, yaw)
                self.get_logger().info(
                    f'odom 对零：x0={x:.3f} y0={y:.3f} yaw0={yaw:.3f}')
            x0, y0, yaw0 = self._origin
            dx = x - x0
            dy = y - y0
            # 将世界坐标系下的位移向量旋转到初始机体朝向的 odom 坐标系（旋转 -yaw0）
            cos_y0 = math.cos(-yaw0)
            sin_y0 = math.sin(-yaw0)
            x = cos_y0 * dx - sin_y0 * dy
            y = sin_y0 * dx + cos_y0 * dy
            yaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0  # two_d 模式，z 压平
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.pose.covariance = cov6([0.05, 0.05, 0.0, 0.0, 0.0, 0.1])  # 初值，看 EKF 收敛再调
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        yaw_speed = float(msg.yaw_speed)
        if abs(yaw_speed) < self.yaw_deadzone:
            yaw_speed = 0.0  # 滤除陀螺零偏，防 EKF 纯积分成自转
        odom.twist.twist.angular.z = yaw_speed
        odom.twist.covariance = cov6([0.1, 0.1, 0.0, 0.0, 0.0, 0.2])  # 初值，看 EKF 收敛再调
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SportOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
