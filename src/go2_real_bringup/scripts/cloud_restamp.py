#!/usr/bin/env python3
# cloud_restamp.py: 对机载激光点云进行本地时间戳重赋（Re-stamping）
#
# 解决宇树 Go2 机载时钟与上位机 PC 未对时（存在几分钟~几小时时差）导致
# pointcloud_to_laserscan 和 RViz 报 "TF extrapolation" 或 "queue is full" 丢弃问题。

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudRestamp(Node):
    def __init__(self):
        super().__init__('cloud_restamp')

        self.declare_parameter('input_topic', '/utlidar/cloud')
        self.declare_parameter('output_topic', 'cloud_synced')
        self.declare_parameter('target_frame', '')  # 为空则保持原 frame_id

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        # 接收机载传感器数据使用 Best Effort QoS
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 发布给本地 ROS 2 节点（pointcloud_to_laserscan）使用 Reliable QoS
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            sub_qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            pub_qos
        )
        self.get_logger().info(f'Cloud Restamp 启动: {input_topic} -> {output_topic}')

    def cloud_callback(self, msg: PointCloud2):
        # 用 PC 本地当前系统时间覆盖消息头时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.target_frame:
            msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
