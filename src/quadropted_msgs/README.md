# quadropted_msgs: 四足运动控制与行为状态自定义消息接口

## 概述 (Overview)
`quadropted_msgs` 定义了 Unitree Go2 仿真运动学控制器中使用的自定义 ROS 2 消息与服务接口。

---

## 接口列表
* `quadropted_msgs/srv/RobotBehaviorCommand`: 机器人起立、坐下、行走状态切换服务。
* `quadropted_msgs/msg/FootState`: 足端触地检测状态。
* `quadropted_msgs/msg/Gait`: 步态参数（步长、周期、占空比等）。
