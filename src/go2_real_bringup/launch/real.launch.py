# real.launch.py：真机总入口（PC 侧）。镜像 l1.launch.py 结构，但没有 Gazebo 相关的一切。
#
# 用法（先完成 P0 组网+DDS+对时）：
#   ros2 launch go2_real_bringup real.launch.py
#   ros2 launch go2_real_bringup real.launch.py rviz:=true
#   # 如果实测点云姿态需要微调：
#   ros2 launch go2_real_bringup real.launch.py rviz:=true lidar_pitch:=2.8782 lidar_roll:=0.0
#
# 启动顺序：
# 1. rsp（use_gazebo:=false 的纯模型）
# 2. utlidar_static_tf：显式发布 base_link -> utlidar_lidar 静态 TF（挂载到 /robot1/tf_static）
# 3. cloud_restamp：订阅机载 /utlidar/cloud，重赋 PC 本地当前时间戳，发布到 /robot1/cloud_synced
# 4. sport 转译：cmd_vel + behavior -> 宇树 sport API
# 5. sport 里程计：sport 状态 -> /robot1/odom
# 6. 点云转激光：cloud_synced -> /robot1/scan
# 7. EKF：odom + IMU -> /robot1/odometry/filtered
# 8. RViz：真机专属 go2_real.rviz（Best Effort QoS + base_link 默认帧）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _nodes(context: LaunchContext, *args, **kwargs):
    # mappings 要普通字符串，必须 perform
    use_sim_time = LaunchConfiguration('use_sim_time')
    ns = 'robot1'
    pkg = get_package_share_directory('go2_real_bringup')

    # 获取静态 TF 参数
    lx = LaunchConfiguration('lidar_x').perform(context)
    ly = LaunchConfiguration('lidar_y').perform(context)
    lz = LaunchConfiguration('lidar_z').perform(context)
    lroll = LaunchConfiguration('lidar_roll').perform(context)
    lpitch = LaunchConfiguration('lidar_pitch').perform(context)
    lyaw = LaunchConfiguration('lidar_yaw').perform(context)

    # TF remap 与仿真一致：节点挂 namespace，统一进 /robot1/tf 与 /robot1/tf_static
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'), ('/odom', 'odometry/filtered')]

    # --- 1. 纯模型（不要 gazebo 插件/摩擦/ros2_control 描述）---
    desc_pkg = get_package_share_directory('go2_description')
    robot_desc = xacro.process_file(
        os.path.join(desc_pkg, 'xacro', 'robot.xacro'),
        mappings={'robot_name': ns,
                  'use_external_lidar': 'false',
                  'use_gazebo': 'false'}).toxml()
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace=ns, output='screen',
               parameters=[{'robot_description': robot_desc,
                            'use_sim_time': use_sim_time}],
               remappings=remappings)

    # --- 2. 静态 TF 广播：将点云坐标系 utlidar_lidar 绑定到 base_link ---
    # 采用显式命名参数，避免位置参数引起的轴向解析混乱
    lidar_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                    name='utlidar_static_tf', namespace=ns, output='screen',
                    arguments=['--x', lx, '--y', ly, '--z', lz,
                               '--roll', lroll, '--pitch', lpitch, '--yaw', lyaw,
                               '--frame-id', 'base_link',
                               '--child-frame-id', 'utlidar_lidar'],
                    remappings=remappings)

    # --- 3. 点云本地时间戳重赋节点：抹平狗机载时钟与 PC 本地的时钟差 ---
    restamp = Node(package='go2_real_bringup', executable='cloud_restamp.py',
                   name='cloud_restamp', namespace=ns, output='screen',
                   parameters=[{'input_topic': '/utlidar/cloud',
                                'output_topic': 'cloud_synced',
                                'target_frame': 'utlidar_lidar',
                                'use_sim_time': use_sim_time}],
                   remappings=remappings)

    # --- 4. 控制转译：cmd_vel + behavior 服务 → sport API（绝对名，不受 namespace 影响）---
    translator = Node(package='go2_real_bringup', executable='sport_translator.py',
                      name='sport_translator', namespace=ns, output='screen',
                      parameters=[os.path.join(pkg, 'config', 'translator.yaml'),
                                  {'use_sim_time': use_sim_time}],
                      remappings=remappings)

    # --- 5. 里程计：sport 状态 → /robot1/odom（只发话题，TF 由 EKF 发）---
    odom = Node(package='go2_real_bringup', executable='sport_odom.py',
                name='sport_odom', namespace=ns, output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=remappings)

    # --- 5b. 多帧累积：cloud_synced x3 滑动窗口 -> cloud_fused ---
    # L1 单帧经高度窗过滤后 /scan 仅 40~80 有效点且奇偶帧互补跳变；
    # 累积 3 帧（约 0.2s）再转激光，有效 beam 数成倍上涨。代价是 0.13s
    # 拖影，建图时限速 0.2m/s（见 P3 规范）。cloud_synced 保留供 RViz 对照。
    accumulate = Node(package='go2_real_bringup', executable='cloud_accumulate.py',
                      name='cloud_accumulate', namespace=ns, output='screen',
                      parameters=[{'input_topic': 'cloud_synced',
                                   'output_topic': 'cloud_fused',
                                   'frames': 3,
                                   'use_sim_time': use_sim_time}],
                      remappings=remappings)

    # --- 6. 点云转激光：从 cloud_fused 读取累积点云并投影至 base_link ---
    laser = Node(package='pointcloud_to_laserscan',
                 executable='pointcloud_to_laserscan_node',
                 name='cloud_to_scan', namespace=ns, output='screen',
                 parameters=[os.path.join(pkg, 'config', 'cloud_to_scan.yaml'),
                             {'use_sim_time': use_sim_time}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                             ('cloud_in', 'cloud_fused'),
                             ('scan', 'scan')])

    # --- 7. EKF：odom + IMU → odometry/filtered（SLAM/Nav2 吃这个；噪声看收敛再调）---
    ekf = Node(package='robot_localization', executable='ekf_node',
               name='ekf_filter_node', namespace=ns, output='screen',
               parameters=[os.path.join(pkg, 'config', 'ekf_real.yaml'),
                           {'use_sim_time': use_sim_time}],
               remappings=remappings)

    # --- 8. 关节状态桥接：订阅机载 /lowstate 发布 /robot1/joint_states，补全腿部 12 关节 TF ---
    joint_bridge = Node(package='go2_real_bringup', executable='joint_state_bridge.py',
                        name='joint_state_bridge', namespace=ns, output='screen',
                        remappings=remappings)

    # --- 9. RViz：启动真机专属配置（Fixed Frame=base_link, Best Effort QoS）---
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                namespace=ns, output='screen',
                arguments=['-d', os.path.join(pkg, 'rviz', 'go2_real.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=remappings,
                condition=IfCondition(LaunchConfiguration('rviz')))

    return [rsp, joint_bridge, lidar_tf, restamp, accumulate, translator, odom, laser, ekf, rviz]


def generate_launch_description():
    return LaunchDescription([
        # 真机默认真实时间；只有回放 rosbag 调试时才设 true
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz（默认 false）'),
        # L1 雷达相对于 base_link 的外参配置
        DeclareLaunchArgument('lidar_x', default_value='0.28945',
                              description='雷达 X 偏移 (m)'),
        DeclareLaunchArgument('lidar_y', default_value='0.0',
                              description='雷达 Y 偏移 (m)'),
        DeclareLaunchArgument('lidar_z', default_value='-0.046825',
                              description='雷达 Z 偏移 (m)'),
        DeclareLaunchArgument('lidar_roll', default_value='0.15',
                              description='雷达 Roll 旋转 (rad，实测标定消除右倾)'),
        DeclareLaunchArgument('lidar_pitch', default_value='3.25',
                              description='雷达 Pitch 旋转 (rad，实测标定拉平俯仰)'),
        DeclareLaunchArgument('lidar_yaw', default_value='2.08',
                              description='雷达 Yaw 旋转 (rad，实测标定消除机身朝向偏左夹角)'),
        OpaqueFunction(function=_nodes),
    ])
