#!/usr/bin/env python3
"""
Target detection node for IBVS.
Subscribes to camera image, extracts red target centroid using HSV thresholding,
and publishes the pixel coordinates to /target_centroid.
Detects both RED target (box) and GREEN feature (ball)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.target_pub = self.create_publisher(Point, '/target_centroid', 10)
        self.feature_pub = self.create_publisher(Point, '/feature_centroid', 10)
        
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])
        
        self.kernel = np.ones((5, 5), np.uint8)
        self.get_logger().info('Target detector initialized.')

    def image_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        mask_r1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_r2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            largest = max(contours_red, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100:
                M = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                self.target_pub.publish(point_msg)
                
                cv2.circle(bgr, (cx, cy), 10, (0, 255, 0), -1)
                self.get_logger().info(f'RED:   u={cx}, v={cy}')

        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.kernel)
        
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_green:
            largest = max(contours_green, key=cv2.contourArea)
            if cv2.contourArea(largest) > 30:
                M = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                self.feature_pub.publish(point_msg)
                
                cv2.circle(bgr, (cx, cy), 10, (0, 0, 255), -1)
                self.get_logger().info(f'GREEN: u={cx}, v={cy}')

        cv2.imwrite('/tmp/detection_output.jpg', bgr)

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
