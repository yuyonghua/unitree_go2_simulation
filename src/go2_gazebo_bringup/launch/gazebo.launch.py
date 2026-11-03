# L1 仿真外层（总入口）：选世界 → 起 Gazebo → 延迟 6 秒 → 调起 l1.launch.py
#
# 用法：
#   ros2 launch go2_gazebo_bringup gazebo.launch.py                           # 默认 warehouse.sdf，只用 L1
#   ros2 launch go2_gazebo_bringup gazebo.launch.py world:=rmuc_2025_world.sdf
#   ros2 launch go2_gazebo_bringup gazebo.launch.py use_external_lidar:=true  # 加挂外置 360° 雷达
#   ros2 launch go2_gazebo_bringup gazebo.launch.py rviz:=true                 # 顺手开 RViz（默认不开）
#   ros2 launch go2_gazebo_bringup gazebo.launch.py x:=1.0 y:=2.0 z:=1.0 yaw:=1.57  # 指定出生位姿
#
# 延迟 6 秒是等 Gazebo 世界加载完再 spawn，原项目也是这个手法（launch.py）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


# 针对不同仿真环境的最佳出生位姿配置字典（避免狗在 straight legs 初始化阶段从过高掉落弹跳翻车或模型穿模）
WORLD_SPAWN_CONFIG = {
    'warehouse.sdf': {'x': '0.0', 'y': '0.0', 'z': '0.45', 'yaw': '0.0'},
    'hospital.sdf': {'x': '0.0', 'y': '15.0', 'z': '0.45', 'yaw': '-1.5708'},       # 医院场景：护士站大厅中央宽阔走廊
    'hospital_full.sdf': {'x': '0.0', 'y': '15.0', 'z': '0.45', 'yaw': '-1.5708'},  # 医院全元素场景：护士站大厅中央宽阔走廊
    'cpr_office.sdf': {'x': '0.0', 'y': '0.0', 'z': '0.45', 'yaw': '0.0'},
    'cafe.world': {'x': '0.0', 'y': '0.0', 'z': '0.65', 'yaw': '0.0'},
    'rmuc_2025_world.sdf': {'x': '0.0', 'y': '0.0', 'z': '0.70', 'yaw': '0.0'},
    'inspection_obstacle_world.sdf': {'x': '0.0', 'y': '0.0', 'z': '0.45', 'yaw': '0.0'},
    'academic_building_world.sdf': {'x': '-15.0', 'y': '-5.0', 'z': '0.45', 'yaw': '1.5708'},
    'benchmark_calibration_world.sdf': {'x': '-7.0', 'y': '7.5', 'z': '0.45', 'yaw': '0.0'},
}

# 标定场景 (benchmark_calibration_world.sdf) 各分赛道一键就位预设位姿
BENCHMARK_TRACKS = {
    'S1': {'x': '-7.0', 'y': '7.5', 'z': '0.45', 'yaw': '0.0'},       # S1: 越障高度敏感性 (3/8/15/30cm)
    'S2': {'x': '-7.0', 'y': '3.25', 'z': '0.45', 'yaw': '0.0'},      # S2: 动态渐缩极窄通道 (1.2m->1.0m->0.8m)
    'S3_12': {'x': '-6.0', 'y': '0.0', 'z': '0.45', 'yaw': '0.0'},    # S3-A: 12° 缓坡道 (平稳减速通过)
    'S3': {'x': '-6.0', 'y': '0.0', 'z': '0.45', 'yaw': '0.0'},       # S3 默认指向 12°
    'S3_18': {'x': '-6.0', 'y': '-1.6', 'z': '0.45', 'yaw': '0.0'},   # S3-B: 18° 临界极限坡
    'S3_26': {'x': '-6.0', 'y': '-3.4', 'z': '0.45', 'yaw': '0.0'},   # S3-C: 26° 致命超限坡 (安全停障拒行)
    'S4': {'x': '-7.0', 'y': '-7.5', 'z': '0.45', 'yaw': '0.0'},      # S4: 致命高坎 vs 绕行缓桥双选线
}


def _gz(context: LaunchContext, *args, **kwargs):
    # world 参数是文件名（如 warehouse.sdf），拼成 worlds/ 下的完整路径
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    wf = os.path.join(get_package_share_directory('go2_gazebo_bringup'), 'worlds', world)
    if not os.path.isfile(wf):
        raise FileNotFoundError(
            f'\n\n[ERROR] Gazebo 世界模型文件未找到: {wf}\n'
            f'可能原因: 1. 文件名拼写错误; 2. 未执行 colcon build 同步到 install 目录。\n'
        )
    gz_flags = f'-s -r -v4 {wf}' if headless else f'-r -v4 {wf}'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags, 'on_exit_shutdown': 'true'}.items())]


def _spawn_robot(context: LaunchContext, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    track = LaunchConfiguration('track').perform(context).strip()
    x_arg = LaunchConfiguration('x').perform(context)
    y_arg = LaunchConfiguration('y').perform(context)
    z_arg = LaunchConfiguration('z').perform(context)
    yaw_arg = LaunchConfiguration('yaw').perform(context)
    enable_ekf_arg = LaunchConfiguration('enable_ekf').perform(context).strip().lower()

    # 1. 出生位姿解析：支持根据世界或赛道名称 (track) 一键自动定位
    if world == 'benchmark_calibration_world.sdf':
        track_key = 'S1' if (track == 'auto' or not track) else track
        cfg = BENCHMARK_TRACKS.get(track_key, BENCHMARK_TRACKS['S1'])
    else:
        cfg = WORLD_SPAWN_CONFIG.get(world, {})

    chosen_x = cfg.get('x', '0.0') if (x_arg == 'auto' or not x_arg) else x_arg
    chosen_y = cfg.get('y', '0.0') if (y_arg == 'auto' or not y_arg) else y_arg
    chosen_z = cfg.get('z', '0.45') if (z_arg == 'auto' or not z_arg) else z_arg
    chosen_yaw = cfg.get('yaw', '0.0') if (yaw_arg == 'auto' or not yaw_arg) else yaw_arg

    # 2. EKF 开关智能仲裁：标定场景及 3D 建图/导航场景下默认由 Point-LIO 发布 odom->base_link，
    #    设为 auto 时自动置为 false，彻底根除双 TF 发布者导致机器狗剧烈闪烁冲突
    if enable_ekf_arg == 'auto':
        is_3d_world = world in ['benchmark_calibration_world.sdf', 'inspection_obstacle_world.sdf', 'academic_building_world.sdf']
        chosen_enable_ekf = 'false' if is_3d_world else 'true'
    else:
        chosen_enable_ekf = enable_ekf_arg

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('go2_gazebo_bringup'), 'launch', 'l1.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'use_external_lidar': LaunchConfiguration('use_external_lidar'),
                          'rviz': LaunchConfiguration('rviz'),
                          'enable_ekf': chosen_enable_ekf,
                          'x': chosen_x,
                          'y': chosen_y,
                          'z': chosen_z,
                          'yaw': chosen_yaw}.items())]


def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'headless', default_value='false',
        description='是否以无 GUI 纯服务端模式运行 Gazebo（避免 X11/OGRE 显存与窗口崩溃）'))
    ld.add_action(DeclareLaunchArgument(
        'use_external_lidar', default_value='false',
        description='是否加载外置 360° 激光雷达，透传给 l1.launch.py'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='false',
        description='是否启动 RViz，透传给 l1.launch.py（配置文件按 use_external_lidar 自动选）'))
    ld.add_action(DeclareLaunchArgument(
        'track', default_value='auto',
        description='标定世界分赛道选择 (S1 | S2 | S3_12 | S3_18 | S3_26 | S4)，自动设定出生位姿'))
    # 出生位姿：不同 world 拓扑与地面高度不同，默认 auto 自动匹配最佳安全位姿
    ld.add_action(DeclareLaunchArgument('x', default_value='auto', description='出生点 x（米），auto 表示按世界/赛道自动配置'))
    ld.add_action(DeclareLaunchArgument('y', default_value='auto', description='出生点 y（米），auto 表示按世界/赛道自动配置'))
    ld.add_action(DeclareLaunchArgument('z', default_value='auto', description='出生点 z（米），auto 表示按世界/赛道自动配置'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='auto', description='出生朝向 yaw（弧度），auto 表示按世界/赛道自动配置'))
    ld.add_action(DeclareLaunchArgument('world', default_value='warehouse.sdf',
                                        description='worlds/ 目录下的世界文件名'))
    ld.add_action(DeclareLaunchArgument('enable_ekf', default_value='auto',
                                        description='是否启动 EKF 节点（auto: 标定/3D场景自动关闭防 TF 冲突；2D场景自动开启）'))
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    ld.add_action(OpaqueFunction(function=_gz))

    # 等 6 秒再 spawn：Gazebo 没起来就 spawn 会失败
    pause = ExecuteProcess(cmd=['sleep', '6'], output='screen')
    ld.add_action(pause)
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=pause,
        on_exit=[OpaqueFunction(function=_spawn_robot)])))
    return ld
