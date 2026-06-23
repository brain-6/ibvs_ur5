#!/usr/bin/env python3
"""
detect_target.py — 订阅相机图像，用 OpenCV 提取红色方块质心坐标
[AI-assisted]
"""

import rclpy                            # ROS2 Python 客户端库
from rclpy.node import Node             # 节点基类
from sensor_msgs.msg import Image       # ROS2 图像消息类型
from cv_bridge import CvBridge          # ROS Image ↔ OpenCV 转换工具
import cv2                              # OpenCV 视觉库
import numpy as np                      # 数组运算


class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')  # 设置节点名称

        self.bridge = CvBridge()             # 创建 cv_bridge 实例

        # 创建订阅者，订阅 Gazebo 相机发布的图像话题
        self.subscription = self.create_subscription(
            Image,                           # 消息类型
            '/camera/image_raw',             # 话题名称
            self.image_callback,             # 回调函数
            10                               # 队列大小
        )

        self.get_logger().info('等待接收图像...')
        self.detected = False                # 标记是否已检测到目标

    def image_callback(self, msg: Image):
        """
        收到图像消息后的回调函数
        msg: sensor_msgs/Image 类型，包含图像数据
        """
        # 1. ROS Image → OpenCV BGR 格式
        #    cv_bridge.imgmsg_to_cv2 将 ROS 消息转为 numpy 数组
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. BGR → HSV 颜色空间
        #    HSV 中 H(色调)区分颜色，S(饱和度)过滤灰色，V(明度)过滤暗色
        #    比 RGB 更适合颜色分割，因为颜色信息集中在 H 通道
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 3. 红色阈值分割（两个区间合并）
        #    红色在 HSV 的 H 通道横跨 0° 和 180°，需要两个区间
        #    低区间：H=0~10（接近 0° 的红色）
        #    高区间：H=170~180（接近 180° 的红色）
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)  # 两个区间取并集

        # 4. 形态学去噪
        #    开运算：先腐蚀后膨胀，去除小噪点
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 5. 查找轮廓 + 计算质心
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 取面积最大的轮廓（最可能是目标方块）
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            # 面积过滤：太小的轮廓忽略（噪声）
            if area > 100:
                # 计算矩（moments），用于求质心
                M = cv2.moments(largest)
                # 质心公式：cx = m10/m00, cy = m01/m00
                # m00 是面积，m10 是 x 方向一阶矩，m01 是 y 方向一阶矩
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 在图像上画红色圆点标记质心
                cv2.circle(bgr, (cx, cy), 10, (0, 255, 0), -1)  # 绿色实心圆
                cv2.putText(bgr, f'({cx}, {cy})', (cx + 15, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 保存标注图片
                cv2.imwrite('/tmp/detection_output.jpg', bgr)

                # 打印检测结果
                self.get_logger().info(f'检测到红色方块！质心坐标: u={cx}, v={cy} (像素)')
                self.get_logger().info(f'轮廓面积: {area} 像素')
                self.get_logger().info(f'标注图片已保存到: /tmp/detection_output.jpg')

                self.detected = True

        if not self.detected:
            # 第一次没检测到时保存原始图像和 mask，方便调试
            cv2.imwrite('/tmp/debug_bgr.jpg', bgr)
            cv2.imwrite('/tmp/debug_mask.jpg', mask)
            self.get_logger().warn('未检测到红色方块，已保存调试图片到 /tmp/')


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
