#!/usr/bin/env python3
# yaw_check.py：静止 10 秒，量化 yaw 漂移到底来自哪一段
#
# 用法（在连狗、跑着 real.launch.py 的机器上）：
#   source env.bash && source install/setup.bash
#   python3 src/go2_real_bringup/scripts/yaw_check.py
#   # 狗保持静止站立 10 秒，不要碰
#
# 判读：
#   - raw yaw_speed 均值明显非零（如 >0.01 rad/s）：机载陀螺零偏，修 sport_odom 死区
#   - raw pose yaw 不漂、filtered 漂：EKF 纯积分发散，修 ekf yaw pose 融合
#   - 两者都不漂但 RViz 里转：问题在 slam 的 map->odom，把 RViz Fixed Frame 切到 odom 验证

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


def get_yaw(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class YawChecker(Node):
    def __init__(self):
        super().__init__('yaw_checker')
        self.raw = []
        self.filt = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/robot1/odom', self.cb_raw, qos)
        self.create_subscription(Odometry, '/robot1/odometry/filtered', self.cb_filt, qos)

    def cb_raw(self, msg: Odometry):
        self.raw.append((time.time(), get_yaw(msg), msg.twist.twist.angular.z))

    def cb_filt(self, msg: Odometry):
        self.filt.append((time.time(), get_yaw(msg)))


def main():
    rclpy.init()
    node = YawChecker()
    print('静止站立 10 秒，开始记录...')
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    import numpy as np
    raw = np.array(node.raw)
    filt = np.array(node.filt)
    print(f'\nraw /robot1/odom: {len(raw)} 帧')
    if len(raw) > 2:
        dt = raw[-1, 0] - raw[0, 0]
        print(f'  pose yaw: {raw[0,1]:.4f} -> {raw[-1,1]:.4f} '
              f'漂移率={(raw[-1,1]-raw[0,1])/dt*180.0/math.pi:.3f} deg/s')
        print(f'  yaw_speed: 均值={raw[:,2].mean():.5f} rad/s '
              f'(={raw[:,2].mean()*180.0/math.pi:.3f} deg/s) 标准差={raw[:,2].std():.5f}')
    print(f'filtered /robot1/odometry/filtered: {len(filt)} 帧')
    if len(filt) > 2:
        dt = filt[-1, 0] - filt[0, 0]
        print(f'  yaw: {filt[0,1]:.4f} -> {filt[-1,1]:.4f} '
              f'漂移率={(filt[-1,1]-filt[0,1])/dt*180.0/math.pi:.3f} deg/s')
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
