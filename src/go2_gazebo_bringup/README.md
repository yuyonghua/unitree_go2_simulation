# go2_gazebo_bringup: Gazebo 仿真总控与软着陆管理

## 概述 (Overview)
`go2_gazebo_bringup` 是 Unitree Go2 四足机器人仿真环境的总控功能包，负责加载物理世界模型、Ignition Gazebo (GZ Sim) 物理引擎桥接、自适应安全软着陆，以及控制器和滤波器的生命周期编排。

---

## 核心特性 (Features)

1. **自适应软着陆 (Soft-Landing Protocol)**：
   - 机器狗初始化时四肢处于笔直僵直（straight legs）状态，若从常规高度（如 0.8m）自由落体极易发生剧烈弹跳翻车。
   - 内置 `WORLD_SPAWN_CONFIG` 矩阵，默认自动将机器狗平稳生成在 `z=0.45m` 触地平衡点，彻底消除落地弹跳翻车。
2. **多场景即开即用 (5 大精选场景)**：
   - 内置仓储、医院标准走廊、医院全要素复杂场景、办公室内景、阶梯大空间 5 款核心物理世界模型。
3. **单发布者 TF 保护**：
   - 支持通过 `enable_ekf:=false` 彻底关闭 `robot_localization` EKF 节点，以便在 3D 模式下由 Point-LIO 50Hz 独占广播 `odom -> base_link`。

---

## 快速使用 (Usage)

```bash
# 1. 默认仓储场景 (warehouse.sdf，软着陆自适应)
ros2 launch go2_gazebo_bringup gazebo.launch.py

# 2. 医院标准巡检场景 (hospital.sdf，自带宽走廊安全位姿)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=hospital.sdf

# 3. 教学楼多层阶梯 3D 场景 (关闭 EKF 避免 TF 冲突，挂载外置 360° 雷达)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=academic_building_world.sdf use_external_lidar:=true enable_ekf:=false

# 4. 无桌面/服务器纯后台运行 (Headless 极速模式，省显存防崩溃)
ros2 launch go2_gazebo_bringup gazebo.launch.py headless:=true
```

---

## 启动参数详表 (Launch Arguments)

| 参数名 | 默认值 | 类型 / 可选值 | 说明 |
| :--- | :--- | :--- | :--- |
| `world` | `warehouse.sdf` | `warehouse.sdf` \| `hospital.sdf` \| `hospital_full.sdf` \| `cpr_office.sdf` \| `academic_building_world.sdf` | 仿真世界文件名称（位于 `worlds/` 目录下） |
| `headless` | `false` | `true` \| `false` | 是否以无 GUI 纯服务端模式运行 Gazebo（CI/服务器/低显存首选） |
| `use_external_lidar`| `false` | `true` \| `false` | 是否加载背部外置 360° 激光雷达（默认 false 仅使用机嘴倒装 L1 雷达） |
| `rviz` | `false` | `true` \| `false` | 是否自动启动 RViz2 监控（配置文件随 `use_external_lidar` 自动切换） |
| `enable_ekf` | `true` | `true` \| `false` | 是否启动 EKF 滤波节点（默认 `true` 开启 2D 滤波；使用 3D 点云里程计如 Point-LIO 时设为 `false` 防 TF 冲突） |
| `x` | `auto` | 浮点数 (米) \| `auto` | 出生点 X 坐标覆盖（`auto` 表示按世界预设自适应软着陆） |
| `y` | `auto` | 浮点数 (米) \| `auto` | 出生点 Y 坐标覆盖（`auto` 表示按世界预设自适应软着陆） |
| `z` | `auto` | 浮点数 (米) \| `auto` | 出生点 Z 高度覆盖（`auto` 默认 `0.45m` 触地保护） |
| `yaw` | `auto` | 弧度制 \| `auto` | 出生朝向偏航角覆盖（`auto` 表示按世界预设方向） |
| `use_sim_time` | `true` | `true` \| `false` | 全局仿真时钟开关（驱动全链路 `/clock` 时间同步） |
