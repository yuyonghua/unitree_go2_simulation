#!/usr/bin/env python3
# check_scan.py: 诊断机载点云、时间戳对齐与 2D LaserScan 转换链路

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ScanChecker(Node):
    def __init__(self):
        super().__init__('scan_checker')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.cloud_count = 0
        self.synced_count = 0
        self.fused_count = 0
        self.scan_count = 0
        self.last_scan_msg = None

        self.create_subscription(PointCloud2, '/utlidar/cloud', self.cb_cloud, sensor_qos)
        self.create_subscription(PointCloud2, '/robot1/cloud_synced', self.cb_synced, sensor_qos)
        self.create_subscription(PointCloud2, '/robot1/cloud_fused', self.cb_fused, sensor_qos)
        self.create_subscription(LaserScan, '/robot1/scan', self.cb_scan, sensor_qos)

        self.timer = self.create_timer(1.0, self.report)
        self.ticks = 0

    def cb_cloud(self, msg):
        self.cloud_count += 1

    def cb_synced(self, msg):
        self.synced_count += 1

    def cb_fused(self, msg):
        self.fused_count += 1
        # 顺带记录累积帧点数（width 即总点数，height=1）
        self.last_fused_width = msg.width

    def cb_scan(self, msg):
        self.scan_count += 1
        self.last_scan_msg = msg

    def report(self):
        self.ticks += 1
        print(f"\n===== [诊断报告 第 {self.ticks} 秒] =====")
        print(f"1. 机载原始点云 /utlidar/cloud 频率:      {self.cloud_count} Hz")
        print(f"2. 同步后点云   /robot1/cloud_synced 频率: {self.synced_count} Hz")
        fused_info = f"{self.fused_count} Hz"
        if hasattr(self, 'last_fused_width'):
            fused_info += f" (约 {self.last_fused_width} 点/帧)"
        print(f"2b. 累积点云     /robot1/cloud_fused 频率: {fused_info}")
        print(f"3. 2D 激光扫描  /robot1/scan 频率:         {self.scan_count} Hz")

        if self.last_scan_msg:
            msg = self.last_scan_msg
            total_beams = len(msg.ranges)
            valid_pts = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            print(f"   [Scan 详细信息] frame_id: '{msg.header.frame_id}'")
            print(f"   [Scan 详细信息] 总激光射线数: {total_beams}")
            print(f"   [Scan 详细信息] 有效障碍物点数: {len(valid_pts)}")
            if valid_pts:
                d_min, d_max = min(valid_pts), max(valid_pts)
                print(f"   [Scan 详细信息] 最近障碍物距离: {d_min:.2f} 米, 最远: {d_max:.2f} 米")
            else:
                print("   [警告] 所有射线均为 inf 或超界，没有一个点被有效保留！")
        else:
            print("   [警告] 未收到任何 /robot1/scan 消息！")

        self.cloud_count = 0
        self.synced_count = 0
        self.fused_count = 0
        self.scan_count = 0


def main():
    rclpy.init()
    node = ScanChecker()
    print("开始监听雷达与激光话题，请保持 real.launch.py 在运行...")
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
