#!/usr/bin/env python3
"""
Target detection node for IBVS.
Subscribes to camera image, extracts red target centroid using HSV thresholding,
and publishes the pixel coordinates to /target_centroid.
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
        
        self.point_pub = self.create_publisher(Point, '/target_centroid', 10)
        
        # Red color in HSV spans across 0 and 180 degrees, requiring two masks
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        self.kernel = np.ones((5, 5), np.uint8)
        
        self.get_logger().info('Target detector initialized.')

    def image_callback(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100:
                M = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                point_msg.z = 0.0
                self.point_pub.publish(point_msg)

                cv2.circle(bgr, (cx, cy), 10, (0, 255, 0), -1)
                cv2.imwrite('/tmp/detection_output.jpg', bgr)
                self.get_logger().info(f'Target: u={cx}, v={cy}, area={cv2.contourArea(largest):.0f}')

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
