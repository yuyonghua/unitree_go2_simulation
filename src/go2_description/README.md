# go2_description: Unitree Go2 四足机器人模型描述与运动学参数

## 概述 (Overview)
`go2_description` 提供了宇树 Unitree Go2 四足机器人的精细三维物理模型、网格几何文件（Meshes）、运动学结构定义与机载前置半固态 L1 激光雷达的外参安装描述。

---

## 核心设计与外参 (Key Specifications)

1. **出厂前置半固态 L1 雷达物理位姿**：
   - 安装于机头嘴部/下颌处：`xyz="0.28945 0.0 -0.046825"`（位于机身中线下方）
   - 物理出厂倒装并向上抬头：`rpy="0.0 2.8782 0.0"`（即 $\pi - 15.09^\circ$，相对于倒装基准向上仰角约 $15.1^\circ$）
   - 原生导出子坐标系：`utlidar_lidar`（激光发射中心）与 `utlidar_imu`（内置 IMU 中心）
2. **纯净模型与仿真解耦 (`use_gazebo`)**：
   - 通过 `use_gazebo:=false` 参数彻底剔除所有 Gazebo 物理插件与虚拟摩擦力参数，供实体真机（`go2_real_bringup`）以极低开销发布纯净 TF 树与 REP-105 坐标变换；
   - 仿真模式（`use_gazebo:=true`）自动挂载 `ros2_control` 12 关节力矩执行器与射线传感器插件。

---

## 快速预览模型 (Visualization)

```bash
# 在 RViz 中单独查看机器人模型几何与关节 TF 结构
ros2 launch go2_description display.launch.py
```
