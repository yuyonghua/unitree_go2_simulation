#!/usr/bin/env python3
# joint_state_bridge.py: 将宇树 Go2 机载电机状态转换为 ROS 2 标准 joint_states
#
# 解决真机环境下缺少 Gazebo joint_state_broadcaster，导致 12 个腿部关节
# 及足端 link (如 lf_foot_link, lf_lower_leg_link 等) 缺失 TF、RViz 中看不到四条腿的问题。

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 尝试导入宇树 LowState 消息类型
try:
    from unitree_go.msg import LowState
    HAS_LOWSTATE = True
except ImportError:
    HAS_LOWSTATE = False

JOINT_NAMES = [
    'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
    'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
    'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
    'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint'
]

# 官方默认站立关节角度 (弧度)
DEFAULT_STAND_Q = [
    0.0, 0.67, -1.3,   # RF: hip, thigh, calf
    0.0, 0.67, -1.3,   # FL: hip, thigh, calf
    0.0, 0.67, -1.3,   # RR: hip, thigh, calf
    0.0, 0.67, -1.3    # RL: hip, thigh, calf
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        self.declare_parameter('lowstate_topic', '/lowstate')
        self.declare_parameter('publish_rate', 50.0)  # 50 Hz

        lowstate_topic = self.get_parameter('lowstate_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        self.current_q = list(DEFAULT_STAND_Q)
        self.has_received_real_data = False

        # 匹配机载传感器专用的 Best Effort QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        if HAS_LOWSTATE:
            self.create_subscription(
                LowState,
                lowstate_topic,
                self.lowstate_callback,
                sensor_qos
            )
            # 同时备用订阅 /lf/lowstate
            self.create_subscription(
                LowState,
                '/lf/lowstate',
                self.lowstate_callback,
                sensor_qos
            )
            self.get_logger().info(f'已订阅宇树低层状态: {lowstate_topic} 及 /lf/lowstate')
        else:
            self.get_logger().warn('未找到 unitree_go/msg/LowState，以默认站立姿势持续发布 joint_states')

        # 50Hz 定时发布 joint_states 给 robot_state_publisher
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('JointStateBridge 初始化完成，开始发布 12 关节 TF 数据')

    def lowstate_callback(self, msg: LowState):
        # 提取 12 个电机当前的实际转角
        if len(msg.motor_state) >= 12:
            self.current_q = [float(msg.motor_state[i].q) for i in range(12)]
            if not self.has_received_real_data:
                self.has_received_real_data = True
                self.get_logger().info('成功接收到 Go2 实机关节状态，切换为真实关节位姿同步')

    def timer_callback(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = self.current_q
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
