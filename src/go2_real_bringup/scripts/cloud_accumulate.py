#!/usr/bin/env python3
# cloud_accumulate.py：多帧点云滑动累积（对付 L1 半固态单帧稀疏）
#
# 背景：L1 非重复扫描（rosette 花瓣），单帧 3900 点投影到 base_link 后经高度窗+
# range 过滤，每束只剩最近点，/scan 有效点 40~80 且奇偶帧来回跳（42↔87）。
# 本节点把连续 N 帧 cloud_synced 直接拼字节合并为一帧 cloud_fused 再喂给
# pointcloud_to_laserscan：同一 beam 上多帧互补，有效 beam 数随之上涨。
# 代价：约 (N-1)/15Hz ≈ 0.13s 运动拖影，建图时必须慢速（≤0.2m/s），与 P3 规范一致。
#
# 实现说明：各帧同 point_step、同字段、height=1，直接拼 data 字节即可，
# 无需逐点 numpy 拷贝；输出 width/row_step 累加，时间戳取最新一帧。

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from collections import deque


class CloudAccumulate(Node):
    def __init__(self):
        super().__init__('cloud_accumulate')
        self.declare_parameter('input_topic', 'cloud_synced')
        self.declare_parameter('output_topic', 'cloud_fused')
        self.declare_parameter('frames', 3)  # 滑动窗口帧数：3x15Hz≈0.2s
        # 注意：use_sim_time 由 rcl 自动声明管理，这里不能再 declare（会抛
        # ParameterAlreadyDeclaredException 导致节点启动即崩溃），launch 透传即生效

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.n_frames = self.get_parameter('frames').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # 发布沿用 Reliable（与 cloud_restamp 一致，下游 BEST_EFFORT 订阅可兼容）
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.buf = deque(maxlen=self.n_frames)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.cb, qos)
        self.pub = self.create_publisher(PointCloud2, output_topic, pub_qos)
        self.get_logger().info(
            f'Cloud Accumulate 启动: {input_topic} x{self.n_frames} -> {output_topic}')

    def cb(self, msg: PointCloud2):
        self.buf.append(msg)
        if len(self.buf) < self.n_frames:
            return  # 窗口未满先不发，避免密度跳变
        first = self.buf[0]
        out = PointCloud2()
        out.header.stamp = msg.header.stamp  # 取最新一帧时间，TF 外推误差最小
        out.header.frame_id = msg.header.frame_id
        out.height = 1
        out.fields = first.fields
        out.is_bigendian = first.is_bigendian
        out.point_step = first.point_step
        out.is_dense = all(m.is_dense for m in self.buf)
        total_width = sum(m.width for m in self.buf)
        out.width = total_width
        out.row_step = first.point_step * total_width
        out.data = b''.join(bytes(m.data) for m in self.buf)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
