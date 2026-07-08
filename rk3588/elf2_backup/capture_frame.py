#!/usr/bin/env python3
import rclpy, cv2, sys, os, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class Capture(Node):
    def __init__(self):
        super().__init__('capture')
        self.bridge = CvBridge()
        self.frame = None
        self.sub = self.create_subscription(Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.cb, 10)
    def cb(self, msg):
        if self.frame is None:
            self.frame = msg
def main():
    rclpy.init()
    node = Capture()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    print('Waiting for frame...', flush=True)
    start = time.time()
    while node.frame is None and time.time() - start < 5:
        executor.spin_once(timeout_sec=0.5)
    if node.frame is None:
        print('TIMEOUT - no frame received', flush=True)
        sys.exit(1)
    frame = node.bridge.imgmsg_to_cv2(node.frame, 'bgr8')
    h, w = frame.shape[:2]
    print(f'Frame received: {w}x{h}', flush=True)
    mean_val = frame.mean()
    print(f'Mean pixel value: {mean_val:.1f}', flush=True)
    if mean_val < 10:
        print('WARNING: Frame is very dark! Check camera lens cap.', flush=True)
    elif mean_val > 240:
        print('WARNING: Frame is overexposed!', flush=True)
    else:
        print('Frame brightness OK', flush=True)
    cv2.imwrite('/home/elf/capture.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    fsize = os.path.getsize('/home/elf/capture.jpg')
    print(f'Saved: /home/elf/capture.jpg ({fsize} bytes)', flush=True)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
