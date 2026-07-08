#!/usr/bin/env python3
"""Simple OpenCV display for motor controller result"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Viewer(Node):
    def __init__(self):
        super().__init__("viewer")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/motor_controller/result", self.cb, 10)
        self.get_logger().info("Viewer ready, press Q to quit")

    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Motor Controller", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(str(e))

def main():
    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
