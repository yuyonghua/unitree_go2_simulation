# quadropted_controller: 仿真四足运动学逆解与关节步态控制器

## 概述 (Overview)
`quadropted_controller` 负责在 Gazebo 物理仿真环境中实现 Go2 机器人的四足运动学正逆解（FK/IK）与 12 关节对角小跑（Trot）步态生成。

---

## 核心接口 (Interfaces)
* **速度输入**：订阅 `/robot1/cmd_vel`（标准 `geometry_msgs/msg/Twist`）
* **状态机服务**：提供 `/robot1/robot_behavior_command`（支持 `up`, `sit`, `walk` 等指令）
* **关节驱动输出**：向 Gazebo `ros2_control` 关节位置控制器发布 12 关节目标角度
