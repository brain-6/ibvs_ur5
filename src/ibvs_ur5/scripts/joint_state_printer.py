#!/usr/bin/env python3
"""
joint_state_printer.py — 订阅 /joint_states 并打印关节角度
[AI-assisted]
"""

import rclpy                          # 导入 ROS2 Python 客户端库
from rclpy.node import Node           # 导入节点基类
from sensor_msgs.msg import JointState  # 导入关节状态消息类型


class JointStatePrinter(Node):        # 继承 Node 基类
    def __init__(self):
        super().__init__('joint_state_printer')  # 设置节点名称

        # 创建订阅者
        # 修正1：话题名 /joint_state → /joint_states（标准话题名，复数）
        self.subscription = self.create_subscription(
            JointState,           # 消息类型
            '/joint_states',      # 订阅的话题名称
            self.callback,        # 消息回调函数
            10                    # 消息队列大小
        )

    def callback(self, msg: JointState):  # 回调函数处理接收到的消息
        # 遍历关节名称和位置列表
        for name, pos in zip(msg.name, msg.position):
            # 修正2：print() → get_logger().info()（ROS2 标准日志方式）
            self.get_logger().info(f'{name}, {pos:.2f} rad')


def main(args=None):
    rclpy.init(args=args)                     # 初始化 ROS2
    node = JointStatePrinter()                # 创建节点
    rclpy.spin(node)                          # 持续运行
    # 修正3：添加资源释放
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
