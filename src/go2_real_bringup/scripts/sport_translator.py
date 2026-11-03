#!/usr/bin/env python3
# sport_translator.py：真机高层控制转译（/robot1/cmd_vel + behavior 服务 → 官方 sport API）
#
# 替换仿真链 cmd_vel_pub + robot_controller_gazebo + joint_group_controller：
# 真机自带机载运动控制器，这里只发高级指令（站/坐/按速度走），不算关节。
# 接口与仿真一致（Nav2 零改动）：订阅 cmd_vel（Twist），服务 sit/up/walk。
# API ID 以官方源码为准：
#   unitree_ros2/example/src/include/common/ros2_sport_client.h
# 组包格式以官方源码为准：
#   unitree_ros2/example/src/src/common/ros2_sport_client.cpp::Move
#   Move 参数为 JSON {"x": vx, "y": vy, "z": vyaw}，发到 /api/sport/request（绝对名，不受命名空间影响）。
#
# 待实测（上机第一天验证）：Move 发一次就行还是要持续发。这里按持续 10Hz 重发实现
# （和 cmd_vel 一个节奏）；若实测单次有效，把 timer 周期调大或只在速度变化时发。

import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
from quadropted_msgs.srv import RobotBehaviorCommand

# API ID（抄官方头文件，不要手写魔法数字）
API_ID_DAMP = 1001
API_ID_BALANCESTAND = 1002
API_ID_STOPMOVE = 1003
API_ID_STANDUP = 1004
API_ID_STANDDOWN = 1005
API_ID_RECOVERYSTAND = 1006
API_ID_MOVE = 1008
API_ID_SIT = 1009
API_ID_SWITCHJOYSTICK = 1027

SPORT_REQUEST_TOPIC = '/api/sport/request'  # 绝对名：官方话题无命名空间


def make_request(api_id: int, parameter: str = '') -> Request:
    req = Request()
    req.header.identity.id = 0
    req.header.identity.api_id = api_id
    req.parameter = parameter
    return req


class SportTranslator(Node):
    def __init__(self):
        super().__init__('sport_translator')
        self.declare_parameter('max_vx', 0.26)       # 与 nav2_l1.yaml 对齐
        self.declare_parameter('max_vy', 0.20)       # 独立横向限幅
        self.declare_parameter('max_vyaw', 1.0)
        self.declare_parameter('republish_rate', 10.0)
        self.declare_parameter('auto_walk', True)    # 收到有效速度自动使能 walk
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_vyaw = self.get_parameter('max_vyaw').value
        self.auto_walk = self.get_parameter('auto_walk').value

        self.req_pub = self.create_publisher(Request, SPORT_REQUEST_TOPIC, 10)
        # cmd_vel 用相对名：随 namespace 解析到 /robot1/cmd_vel
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_service(RobotBehaviorCommand, 'robot_behavior_command',
                            self.behavior_callback)

        self.walk_enabled = False  # 是否处于行走使能状态
        self.was_moving = False    # 上一周期是否在移动（用于发一次 StopMove）
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        self.last_cmd_time = 0.0   # 看门狗时间戳
        self._last_nowalk_warn = 0.0
        rate = self.get_parameter('republish_rate').value
        self.create_timer(1.0 / rate, self.republish)
        self.get_logger().info('SportTranslator 启动完成，监听 /robot1/cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        # 钳制：仿真步态限制沿用到真机，超了直接砍；横向速度独立限幅
        self.last_vx = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        self.last_vy = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        self.last_vyaw = max(-self.max_vyaw, min(self.max_vyaw, msg.angular.z))
        self.last_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def republish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # 看门狗超时保护：超过 0.5s 未收到 cmd_vel 指令，自动归零防飞车
        if self.last_cmd_time > 0.0 and (now - self.last_cmd_time > 0.5):
            self.last_vx = 0.0
            self.last_vy = 0.0
            self.last_vyaw = 0.0

        has_motion = (abs(self.last_vx) + abs(self.last_vy) + abs(self.last_vyaw)) > 1e-3

        if not self.walk_enabled:
            if has_motion:
                if self.auto_walk:
                    self.get_logger().info('收到有效导航速度指令，自动切入 BalanceStand 并使能 walk 行走')
                    self.req_pub.publish(make_request(API_ID_BALANCESTAND))
                    self.walk_enabled = True
                else:
                    if now - self._last_nowalk_warn > 5.0:
                        self._last_nowalk_warn = now
                        self.get_logger().warn(
                            '收到 cmd_vel 但未切 walk 模式，速度被丢弃！先调 walk 服务：'
                            " ros2 service call /robot1/robot_behavior_command"
                            " quadropted_msgs/srv/RobotBehaviorCommand \"{command: 'walk'}\"")
                    return
            else:
                return

        if has_motion:
            param = json.dumps({'x': self.last_vx, 'y': self.last_vy, 'z': self.last_vyaw})
            self.req_pub.publish(make_request(API_ID_MOVE, param))
            self.was_moving = True
        else:
            # 停止时仅发送一次 StopMove (1003)，切入平衡站立，避免 10Hz 轰炸 Move 0 导致运控死锁
            if self.was_moving:
                self.req_pub.publish(make_request(API_ID_STOPMOVE))
                self.was_moving = False

    def behavior_callback(self, request, response):
        command = request.command.lower()
        self.get_logger().info(f'收到行为指令: {command}')
        if command == 'sit':
            self.walk_enabled = False
            self.was_moving = False
            self.req_pub.publish(make_request(API_ID_STOPMOVE))
            self.req_pub.publish(make_request(API_ID_SIT))
            response.success = True
            response.message = '机器人坐了下来。'
        elif command in ['up', 'stand']:
            self.walk_enabled = False
            self.was_moving = False
            self.req_pub.publish(make_request(API_ID_STANDUP))
            response.success = True
            response.message = '机器人站了起来（位置站立）。'
        elif command in ['balance', 'balancestand']:
            self.walk_enabled = False
            self.was_moving = False
            self.req_pub.publish(make_request(API_ID_BALANCESTAND))
            response.success = True
            response.message = '机器人进入动态平衡站立模式。'
        elif command == 'walk':
            # 切换为 walk 时，发 BalanceStand 解除四肢刚性锁定，切入动态平衡
            self.req_pub.publish(make_request(API_ID_BALANCESTAND))
            self.walk_enabled = True
            self.was_moving = False
            response.success = True
            response.message = '机器人已解除锁定，进入 walk 动态行走准备状态。'
        elif command == 'damp':
            self.walk_enabled = False
            self.was_moving = False
            self.req_pub.publish(make_request(API_ID_DAMP))
            response.success = True
            response.message = '机器人进入阻尼状态。'
        else:
            response.success = False
            response.message = f'未知命令: {command}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SportTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
