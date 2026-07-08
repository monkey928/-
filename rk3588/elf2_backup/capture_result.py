import rclpy, cv2, sys, os, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
class Capture(Node):
    def __init__(self):
        super().__init__('capres')
        self.bridge = CvBridge()
        self.frame = None
        self.last_target = None
        self.sub_img = self.create_subscription(Image, '/trash_detector/result', self.cb_img, 10)
        self.sub_tgt = self.create_subscription(Float32MultiArray, '/trash_detector/target', self.cb_tgt, 10)
    def cb_img(self, msg): 
        if self.frame is None: self.frame = msg
    def cb_tgt(self, msg):
        self.last_target = msg.data
def main():
    rclpy.init()
    node = Capture()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    print('Waiting for detection result...', flush=True)
    start = time.time()
    while node.frame is None and time.time() - start < 8:
        executor.spin_once(timeout_sec=0.5)
    if node.frame is None:
        print('TIMEOUT - no result frame', flush=True)
        sys.exit(1)
    frame = node.bridge.imgmsg_to_cv2(node.frame, 'bgr8')
    h, w = frame.shape[:2]
    fs = os.path.getsize('/home/elf/capture.jpg') if os.path.exists('/home/elf/capture.jpg') else 0
    cv2.imwrite('/home/elf/detection_result.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    nfs = os.path.getsize('/home/elf/detection_result.jpg')
    print(f'Result frame: {w}x{h}, {nfs} bytes', flush=True)
    if node.last_target:
        print(f'Last target: {node.last_target}', flush=True)
    else:
        print('No target detected in this frame', flush=True)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
