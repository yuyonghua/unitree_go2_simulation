# go2_real_bringup: Unitree Go2 实体真机底座与 SDK2 桥接驱动

## 概述 (Overview)
`go2_real_bringup` 是面向 Unitree Go2 实体机器狗的真机部署功能包。
通过宇树官方 SDK2 CycloneDDS 通信协议，在 PC 端将标准 ROS 2 控制与感知生态与机器狗机载电脑无缝桥接，实现 Sim-to-Real 的零代码撕裂迁移。

---

## 核心架构与节点 (Architecture & Nodes)

1. **`sport_translator.py` (运动控制转译)**：
   - 订阅 `/robot1/cmd_vel`（标准 `geometry_msgs/msg/Twist`）和行为服务 `/robot1/robot_behavior_command`；
   - 动态转译为宇树机载运动控制 API（JSON Move 协议）并下发至 `/api/sport/request`。
2. **`sport_odom.py` (机载状态解析与里程计)**：
   - 监听机载 CycloneDDS 高频广播 `lf/sportmodestate`；
   - 解析机载惯导融合里程计并发布为标准 `/robot1/odom`。
3. **`cloud_restamp.py` (时钟重对齐)**：
   - 解决真机与车载 PC 纳秒级时钟未硬件同步问题，接收机载点云并对齐本地 ROS 2 时间戳。
4. **`pointcloud_to_laserscan` (2D 激光切片)**：
   - 将前置 L1 3D 点云沿高度切片过滤地面噪波，生成标准 2D `/robot1/scan`。

---

## 快速使用 (Usage)

> **前置要求**：完成网卡配置、DDS 配置文件（`cyclonedds_real.xml`）与物理对时。详见 [`docs/GO2-真机部署指南.md`](../../docs/GO2-真机部署指南.md)。

```bash
# 1. 启动真机 Bringup 驱动底座
ros2 launch go2_real_bringup real.launch.py

# 2. 顺手启动真机专属 RViz 可视化
ros2 launch go2_real_bringup real.launch.py rviz:=true

# 3. 实体真机 2D 导航
ros2 launch go2_nav nav.launch.py mode:=2d use_sim_time:=false map:=<真机地图路径> rviz:=true

# 4. 实体真机 3D 先验重定位导航
ros2 launch go2_nav nav.launch.py mode:=3d use_sim_time:=false pcd:=<真机点云路径> continuous_refine:=true rviz:=true
```
