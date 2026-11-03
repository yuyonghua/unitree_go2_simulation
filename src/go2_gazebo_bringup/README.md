# go2_gazebo_bringup: Gazebo 仿真总控与软着陆管理

## 概述 (Overview)
`go2_gazebo_bringup` 是 Unitree Go2 四足机器人仿真环境的总控功能包，负责加载物理世界模型、Ignition Gazebo (GZ Sim) 物理引擎桥接、自适应安全软着陆，以及控制器和滤波器的生命周期编排。

---

## 核心特性 (Features)

1. **自适应软着陆 (Soft-Landing Protocol)**：
   - 机器狗初始化时四肢处于笔直僵直（straight legs）状态，若从常规高度（如 0.8m）自由落体极易发生剧烈弹跳翻车。
   - 内置 `WORLD_SPAWN_CONFIG` 矩阵，默认自动将机器狗平稳生成在 `z=0.45m` 触地平衡点，彻底消除落地弹跳翻车。
2. **基准赛道一键就位 (`track:=S1|S2|S3|S4`)**：
   - 内置 `BENCHMARK_TRACKS`，支持在 `benchmark_calibration_world.sdf` 中一键将机器狗放置于各实验分赛道起点。
3. **单发布者 TF 保护**：
   - 支持通过 `enable_ekf:=false` 彻底关闭 `robot_localization` EKF 节点，以便在 3D 模式下由 Point-LIO 50Hz 独占广播 `odom -> base_link`。

---

## 快速使用 (Usage)

```bash
# 1. 默认仓库世界 (warehouse.sdf)
ros2 launch go2_gazebo_bringup gazebo.launch.py

# 2. 巡检越障世界 (关闭 EKF 以配合 3D SLAM 或 MPPI 导航)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=inspection_obstacle_world.sdf enable_ekf:=false

# 3. 基准标定世界 (指定分赛道 S1 越障高度敏感性)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=benchmark_calibration_world.sdf track:=S1 enable_ekf:=false
```

---

## 常用参数 (Launch Arguments)

| 参数名 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `world` | `warehouse.sdf` | 仿真世界文件名称（位于 `worlds/` 下） |
| `enable_ekf` | `true` | 是否启动 2D 轮式+IMU EKF 滤波节点（3D 模式必须设为 `false`） |
| `use_external_lidar` | `false` | 是否挂载外置 360° 激光雷达（默认仅使用前置 L1） |
| `rviz` | `false` | 是否同时拉起基础 RViz2 监控面板 |
| `track` | `''` | 基准分赛道选择（如 `S1`, `S2`, `S3_18`, `S4`） |
| `x / y / z / yaw` | `auto` | 出生点位姿覆盖（默认按场景自适应匹配） |
