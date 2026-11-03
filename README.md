# Unitree Go2 机器人机体资产与虚实硬件抽象基座 (`go2_base_ws`)

本项目是宇树 **Unitree Go2** 四足机器人的**基础机体资产与虚实底盘硬件抽象层（Level 0 HAL）**。
作为底层基础工作空间，它向上为**学术算法研发仓（`go2_ws`）**与**工业巡检工程仓（`unitree_ws`）**提供统一、解耦的机体物理模型、Gazebo 仿真世界及真机 SDK2 桥接驱动。

---

## 🏗️ 架构定位与功能包矩阵

```
go2_base_ws/src/
├── go2_description        # [机体描述] 12 关节运动学模型、Mesh 几何、机嘴倒装抬头 L1 雷达外参 (rpy="0.0 2.8782 0.0")
├── go2_gazebo_bringup     # [仿真总控] Gazebo Fortress 世界模型 (9大世界)、软着陆 (z=0.45m)、GZ Bridge 桥接
├── go2_real_bringup       # [真机底座] 官方 SDK2 桥接驱动、时钟时间戳重赋、雷达外参静态 TF、关节状态桥接
├── quadropted_controller  # [仿真运控] 四足运动学逆解 (IK) 与高抬腿步态生成器 (robot_controller_gazebo)
└── quadropted_msgs        # [运控消息] 步态与底盘行为控制自定义 Msg/Srv IDL
```

---

## ⚡ 极速构建与使用指南

### 1. 环境变量加载
```bash
cd /home/user/git/yyh/go2_base_ws
source env.sh
```

### 2. 编译工作空间
```bash
colcon build --symlink-install --parallel-workers 4
```

### 3. 运行物理仿真 (Gazebo Fortress)
```bash
# 启动默认仓库场景 (软着陆 z=0.45m，前置点云 /robot1/scan/points 就绪)
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=warehouse.sdf

# 启动 ASTM 连续越障标准化基准场景
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=inspection_obstacle_world.sdf

# 启动 1200 m² 医院巡检场景
ros2 launch go2_gazebo_bringup gazebo.launch.py world:=hospital.sdf
```

### 4. 运行实体机器狗硬件底座
```bash
# 连接真实 Go2 底盘 (需先完成网段 192.168.123.99 配置)
ros2 launch go2_real_bringup real.launch.py
```
