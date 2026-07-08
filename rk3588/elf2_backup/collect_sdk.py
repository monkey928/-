import rclpy, cv2, os, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SAVE_DIR = "/home/elf/dataset_sdk"
TOTAL = 300
os.makedirs(SAVE_DIR, exist_ok=True)

class Collector(Node):
    def __init__(self):
        super().__init__("collector")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image,
            "/ascamera_hp60c/camera_publisher/rgb0/image", self.cb, 10)
        self.count = 0
        self.last = 0
    def cb(self, msg):
        now = time.time()
        if now - self.last < 0.5 or self.count >= TOTAL:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return
        path = os.path.join(SAVE_DIR, "paper_%04d.jpg" % self.count)
        cv2.imwrite(path, frame)
        self.count += 1
        self.last = now
        if self.count % 25 == 0:
            print("  [%d/%d]" % (self.count, TOTAL))
        if self.count >= TOTAL:
            print("Done! %d images" % self.count)
            rclpy.shutdown()

rclpy.init()
rclpy.spin(Collector())
