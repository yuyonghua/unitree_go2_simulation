# Unitree Go2 机器人机体资产与虚实硬件抽象基座 (`go2_base_ws`)

本项目是宇树 **Unitree Go2** 四足机器人的**基础机体资产与虚实底盘硬件抽象层（Level 0 HAL）**。
作为底层基础工作空间，它向上为**学术算法研发仓（`go2_ws`）**与**工业巡检工程仓（`unitree_ws`）**提供统一、解耦的机体物理模型、Gazebo 仿真世界及真机 SDK2 桥接驱动。

---

## 🏗️ 架构定位与功能包矩阵

```
go2_base_ws/src/
├── go2_description        # [机体描述] 12 关节运动学模型、Mesh 几何、机嘴倒装抬头 L1 雷达外参 (rpy="0.0 2.8782 0.0")
├── go2_gazebo_bringup     # [仿真总控] Gazebo Fortress 世界模型 (5大精选世界)、软着陆 (z=0.45m)、GZ Bridge 桥接
├── go2_real_bringup       # [真机底座] 官方 SDK2 桥接驱动、时钟时间戳重赋、雷达外参静态 TF、关节状态桥接
├── quadropted_controller  # [仿真运控] 四足运动学逆解 (IK) 与高抬腿步态生成器 (robot_controller_gazebo)
└── quadropted_msgs        # [运控消息] 步态与底盘行为控制自定义 Msg/Srv IDL
```

---

## ⚡ 极速构建与使用指南

### 1. 环境变量加载
```bash
cd /home/user/git/yyh/go2_base_ws
source env.bash
```

### 2. 编译工作空间
```bash
colcon build --symlink-install --parallel-workers 4
```

---

## 🎮 仿真总控 (`go2_gazebo_bringup`) 参数与运行模式

仿真入口统一通过 `ros2 launch go2_gazebo_bringup gazebo.launch.py` 启动。该 Launch 文件支持丰富的可选配置参数，可根据不同算力环境、传感器配置与实验需求自由组合：

### 1. Launch 可选参数总览

| 参数名 | 默认值 | 类型 / 可选值 | 说明 |
| :--- | :--- | :--- | :--- |
| `world` | `warehouse.sdf` | `warehouse.sdf`<br>`hospital.sdf`<br>`hospital_full.sdf`<br>`cpr_office.sdf`<br>`academic_building_world.sdf` | 仿真物理世界文件名称（位于 `go2_gazebo_bringup/worlds/` 下） |
| `headless` | `false` | `true` \| `false` | 是否开启纯服务端无 GUI 渲染模式（适合 CI、无桌面的 Docker 容器或节省 GPU 显存） |
| `use_external_lidar` | `false` | `true` \| `false` | 是否在机器狗背部加挂外置 360° 激光雷达（默认 false，仅使用前置机嘴倒装 L1 雷达） |
| `rviz` | `false` | `true` \| `false` | 是否在启动仿真的同时拉起 RViz2 监控（会自动根据是否挂载外置雷达选用对应的显示配置） |
| `enable_ekf` | `true` | `true` \| `false` | 是否启用 `robot_localization` 2D EKF 滤波节点。<br>默认 `true`（2D 轮式+IMU 融合发布 `odom->base_link`）；跑 3D SLAM（如 Point-LIO）或全地形场景时建议显式设为 `false` 防止 TF 发布冲突 |
| `x` | `auto` | 浮点数 (米) \| `auto` | 机器狗生成（Spawn）的 X 轴坐标。`auto` 时自动根据世界预设加载防翻车最佳出生位姿 |
| `y` | `auto` | 浮点数 (米) \| `auto` | 机器狗生成的 Y 轴坐标。`auto` 时自动匹配世界预设 |
| `z` | `auto` | 浮点数 (米) \| `auto` | 机器狗生成的 Z 轴触地高度。`auto` 默认 `0.45m` 软着陆（避免四肢僵直落地撞地弹跳翻车） |
| `yaw` | `auto` | 浮点数 (弧度) \| `auto` | 机器狗生成的初始朝向偏航角 |
| `use_sim_time` | `true` | `true` \| `false` | 是否使用 Gazebo 仿真时间 `/clock`（ROS 2 物理仿真标配，必须保持 true） |

---

### 2. 精选 5 大仿真世界模型

| 世界文件名 | 场景特点与适用领域 | 预设出生位姿 (`x, y, z, yaw`) |
| :--- | :--- | :--- |
| `warehouse.sdf` | 默认仓储物流世界，开阔平整，适合 2D SLAM、自主建图与 Nav2 路径规划基准调试 | `0.0, 0.0, 0.45, 0.0` |
| `hospital.sdf` | 约 1200 m² 标准医院场景，包含候诊大厅、分诊台及标准走廊，适合服务机器人巡检测试 | `0.0, 15.0, 0.45, -1.5708` |
| `hospital_full.sdf` | 医院全要素高密度场景，富含病床、仪器台、隔帘等密集静态障碍物，适合严苛避障验证 | `0.0, 15.0, 0.45, -1.5708` |
| `cpr_office.sdf` | 典型办公室内景，包含隔间、工位桌椅、过道等室内窄通道拓扑结构 | `0.0, 0.0, 0.45, 0.0` |
| `academic_building_world.sdf` | 高校教学楼大尺度 3D 场景，含楼梯、上下斜坡与多层悬挑连廊，适合 3D SLAM 与全地形运动学评估 | `-15.0, -5.0, 0.45, 1.5708` |

---

### 3. 典型运行命令与组合范例

#### ① 默认场景极速启动 (仓储环境 + 内置 L1 点云)
```bash
# 启动默认仓储世界，自动软着陆 (z=0.45m)，点云转 2D 激光 (/robot1/scan) 与里程计自动就绪
ros2 launch go2_gazebo_bringup gazebo.launch.py
```

#### ② 切换物理世界 (如医院大场景)
```bash
# 载入 1200 m² 医院巡检环境，自动将机器狗投放于护士站大厅主走廊
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=hospital.sdf
```

#### ③ 挂载背部外置 360° 激光雷达 + 同步开启 RViz2
```bash
# 开启外置 Velodyne 雷达外设，并启动预配置好的 RViz 监控面板进行直观可视化
ros2 launch go2_gazebo_bringup gazebo.launch.py use_external_lidar:=true rviz:=true
```

#### ④ 无头服务器模式 (Headless / 低显存 / 远程 SSH 模式)
```bash
# 不打开 Gazebo GUI 窗口，纯后台启动物理引擎和传感器桥接，显著降低 GPU 显存与 CPU 占用
ros2 launch go2_gazebo_bringup gazebo.launch.py headless:=true
```

#### ⑤ 3D 复杂全地形场景 (教学楼多层连廊与楼梯，关闭 2D EKF)
```bash
# 进入教学楼大场景，挂载 360° 雷达并关闭 2D EKF，将 odom->base_link 独占交给 3D SLAM (如 Point-LIO)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=academic_building_world.sdf use_external_lidar:=true enable_ekf:=false
```

#### ⑥ 自定义机器狗出生坐标与朝向
```bash
# 手动指定出生点 (x=2.5, y=-1.0, z=0.45, 朝向 yaw=1.57 弧度)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=warehouse.sdf x:=2.5 y:=-1.0 z:=0.45 yaw:=1.57
```

---

## 🐕 实体机器狗硬件底座 (`go2_real_bringup`)

若需要连接真实的 Unitree Go2 实体狗底盘：

```bash
# 1. 确保本机有线网卡已静态配置在 192.168.123.99/24 网段，且能 ping 通 192.168.123.161
ping 192.168.123.161

# 2. 启动真机底座（含 SDK2 桥接驱动、时钟时间戳重赋、雷达外参静态 TF 与关节状态监控）
ros2 launch go2_real_bringup real.launch.py
```

---

## 🙏 致谢与参考开源项目 (Acknowledgements & References)

本项目在架构设计、仿真环境构建及四足运控适配过程中，参考并借鉴了以下优秀的开源社区项目与数字资产，特此向原作者与维护团队致以真诚谢意：

### 1. 机器人仿真框架与运控算法
| 开源项目 | 原始作者 / 机构 | 开源协议 | 主要借鉴与集成说明 |
| :--- | :--- | :--- | :--- |
| [**`ROS2-Gazebo-GO2`**](https://github.com/yanyuze1/ROS2-Gazebo-GO2) | yanyuze1 | Apache-2.0 | ROS 2 Humble + Ignition Gazebo (Fortress) 基础仿真架构、GZ-ROS 话题桥接与 Go2 机体资产整合方案 |
| [**`spotMicro`**](https://github.com/mike4192/spotMicro) | mike4192 | MIT | 四足机器人逆运动学（Inverse Kinematics）与 Trot Gait 对角小跑步态核心算法 |
| [**`open_quadruped`**](https://github.com/lnotspotl/open_quadruped) | lnotspotl, abutalipovvv | Apache-2.0 | 四足机体状态机、高抬腿 Crawl Gait 爬行步态生成器及 Gazebo 控制器封装 |

### 2. Gazebo 物理仿真世界与场景资产
| 开源项目 | 维护组织 | 开源协议 | 集成场景与世界模型文件 |
| :--- | :--- | :--- | :--- |
| [**`aws-robomaker-hospital-world`**](https://github.com/aws-robotics/aws-robomaker-hospital-world) | AWS Robotics | Apache-2.0 | `hospital.sdf`、`hospital_full.sdf`（1200 m² 医院全要素场景与医疗设备资产） |
| [**`aws-robomaker-small-warehouse-world`**](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) | AWS Robotics | Apache-2.0 | `warehouse.sdf`（现代室内智能仓储物流世界、货架与货箱模型） |
| [**`cpr_gazebo`**](https://github.com/clearpathrobotics/cpr_gazebo) | Clearpath Robotics | BSD-3-Clause | `cpr_office.sdf`（多隔间办公室内景仿真环境与狭窄通道拓扑） |

### 3. 宇树官方机体资产与硬件通信
| 依赖项目 / 资产源 | 所属厂商 | 用途说明 |
| :--- | :--- | :--- |
| [**`unitree_ros`**](https://github.com/unitreerobotics/unitree_ros) | 杭州宇树科技有限公司 (Unitree Robotics) | Go2 机体 URDF 几何、关节动力学参数与官方高精度 Mesh 渲染资产 |
| [**`unitree_sdk2`**](https://github.com/unitreerobotics/unitree_sdk2) | 杭州宇树科技有限公司 (Unitree Robotics) | 实体 Unitree Go2 底盘 DDS 低时延运控协议与真机通信驱动接口 |

