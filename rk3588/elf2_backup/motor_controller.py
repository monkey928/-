#!/usr/bin/env python3
"""ROS2 Motor Controller — uses trash_detector targets, sends binary protocol to STM32"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial
import time
import threading
import struct


# === Binary Protocol Constants ===
HEAD1, HEAD2 = 0xAA, 0x55
CMD_VELOCITY = 0x01
CMD_STOP = 0x03
CMD_EMERGENCY = 0x04
CMD_CLEAR = 0x05
TEL_STATUS = 0x81


def make_frame(cmd_type, payload=b""):
    length = len(payload)
    cksum = (cmd_type + length + sum(payload)) & 0xFF
    return bytes([HEAD1, HEAD2, cmd_type, length]) + payload + bytes([cksum])


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # === Serial ===
        self.ser = serial.Serial("/dev/ttyS9", 115200, timeout=0.1)
        self.ser_lock = threading.Lock()
        self.get_logger().info("Serial /dev/ttyS9 opened")

        # === Subscribers ===
        self.declare_parameter("camera_ns", "ascamera_hp60c")
        self.bridge = CvBridge()
        ns = self.get_parameter("camera_ns").value

        # Trash detector target [cx, cy, distance_m, area]
        self.sub_target = self.create_subscription(
            Float32MultiArray, "/trash_detector/target", self.target_callback, 10)
        # RGB for display overlay
        self.sub_rgb = self.create_subscription(
            Image, f"/{ns}/camera_publisher/rgb0/image", self.rgb_callback, 10)

        # === Publisher ===
        self.pub_result = self.create_publisher(Image, "/motor_controller/result", 10)

        # === Telemetry state ===
        self.tel_left_speed = 0
        self.tel_right_speed = 0
        self.tel_x = 0
        self.tel_y = 0
        self.tel_theta = 0
        self.tel_battery = 0
        self.tel_mode = 0
        self.tel_stall = 0
        self.tel_lock = threading.Lock()

        # === Target from trash_detector ===
        self.target_cx = 0.0
        self.target_cy = 0.0
        self.target_dist = -1.0
        self.target_area = 0.0
        self.target_valid = False
        self.target_lock = threading.Lock()

        # === Control state ===
        self.last_frame = None
        self.frame_lock = threading.Lock()
        self.current_v = 0
        self.current_w = 0
        self.target_lost_count = 0
        self.scan_dir = 1
        self.frame_count = 0

        # === Parameters ===
        self.declare_parameter("max_speed_mm_s", 400)
        self.declare_parameter("turn_speed_mrad_s", 2000)
        self.declare_parameter("search_speed_mrad_s", 1500)
        self.declare_parameter("stop_distance", 0.3)
        self.declare_parameter("approach_distance", 0.8)
        self.declare_parameter("center_tolerance", 60)

        # === Telemetry reader thread ===
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.reader_thread.start()

        self.get_logger().info("Motor controller ready")

    # ========== Serial Protocol ==========

    def serial_reader(self):
        buf = b""
        while self.reader_running:
            try:
                if self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                while len(buf) >= 20:
                    idx = buf.find(bytes([HEAD1, HEAD2]))
                    if idx < 0:
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < 20:
                        break
                    if buf[2] == TEL_STATUS and buf[3] == 15:
                        if len(buf) >= 20:
                            self.parse_telemetry(buf[4:19])
                            buf = buf[20:]
                        else:
                            break
                    else:
                        buf = buf[2:]
                if len(buf) > 512:
                    buf = buf[-256:]
                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def parse_telemetry(self, payload):
        try:
            with self.tel_lock:
                self.tel_left_speed = struct.unpack('<h', payload[0:2])[0]
                self.tel_right_speed = struct.unpack('<h', payload[2:4])[0]
                self.tel_x = struct.unpack('<h', payload[4:6])[0]
                self.tel_y = struct.unpack('<h', payload[6:8])[0]
                self.tel_theta = struct.unpack('<h', payload[8:10])[0]
                self.tel_battery = struct.unpack('<H', payload[10:12])[0]
                self.tel_mode = payload[12]
                self.tel_stall = payload[13]
        except Exception:
            pass

    def write_frame(self, frame):
        with self.ser_lock:
            try:
                self.ser.write(frame)
                self.ser.flush()
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

    def send_velocity(self, v_mm_s, w_mrad_s):
        payload = struct.pack('<hh', int(v_mm_s), int(w_mrad_s))
        self.write_frame(make_frame(CMD_VELOCITY, payload))

    def send_stop(self):
        self.write_frame(make_frame(CMD_STOP, b""))

    # ========== Callbacks ==========

    def target_callback(self, msg):
        """Receive detection from trash_detector: [cx, cy, distance_m, area]"""
        if len(msg.data) >= 3:
            with self.target_lock:
                self.target_cx = msg.data[0]
                self.target_cy = msg.data[1]
                self.target_dist = msg.data[2]
                self.target_area = msg.data[3] if len(msg.data) >= 4 else 0.0
                self.target_valid = True

    def rgb_callback(self, msg):
        self.frame_count += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        h, w = frame.shape[:2]
        cx_img = w // 2

        # Get latest target
        with self.target_lock:
            tx = self.target_cx
            ty = self.target_cy
            dist = self.target_dist
            valid = self.target_valid

        # === Visual servoing ===
        max_spd = self.get_parameter("max_speed_mm_s").value
        turn_spd = self.get_parameter("turn_speed_mrad_s").value
        search_spd = self.get_parameter("search_speed_mrad_s").value
        stop_dist = self.get_parameter("stop_distance").value
        approach_dist = self.get_parameter("approach_distance").value
        center_tol = self.get_parameter("center_tolerance").value

        status = "IDLE"
        color = (200, 200, 200)

        if valid:
            self.target_lost_count = 0
            error_x = tx - cx_img

            # Draw target
            cv2.circle(frame, (int(tx), int(ty)), 30, (0, 255, 0), 2)
            cv2.line(frame, (cx_img, h // 2), (int(tx), int(ty)), (255, 0, 0), 1)

            if dist > 0 and dist < stop_dist:
                self.current_v, self.current_w = 0, 0
                self.send_stop()
                status = f"STOP {dist:.2f}m"
                color = (0, 255, 0)
            elif abs(error_x) < center_tol:
                v = max_spd if dist < 0 or dist > approach_dist else max_spd // 2
                self.current_v, self.current_w = v, 0
                self.send_velocity(v, 0)
                status = f"FWD {v}mm/s ({dist:.2f}m)"
                color = (255, 0, 0)
            elif error_x > 0:
                self.current_v, self.current_w = 0, turn_spd
                self.send_velocity(0, turn_spd)
                status = "TURN R"
                color = (0, 255, 255)
            else:
                self.current_v, self.current_w = 0, -turn_spd
                self.send_velocity(0, -turn_spd)
                status = "TURN L"
                color = (0, 255, 255)
        else:
            self.target_lost_count += 1
            if self.target_lost_count > 10:
                w_cmd = search_spd * self.scan_dir
                self.current_v, self.current_w = 0, w_cmd
                self.send_velocity(0, w_cmd)
                if self.target_lost_count > 60:
                    self.target_lost_count = 10
                    self.scan_dir *= -1
                status = "SEARCH"
                color = (128, 128, 128)
            else:
                status = "WAIT"
                color = (200, 200, 200)

        # === HUD ===
        cv2.line(frame, (cx_img - 15, h // 2), (cx_img + 15, h // 2), (0, 255, 255), 1)
        cv2.line(frame, (cx_img, h // 2 - 15), (cx_img, h // 2 + 15), (0, 255, 255), 1)
        cv2.putText(frame, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
        cv2.putText(frame, f"v={self.current_v}mm/s w={self.current_w}mrad/s",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        with self.tel_lock:
            cv2.putText(frame,
                        f"L:{self.tel_left_speed} R:{self.tel_right_speed}mm/s bat:{self.tel_battery}mV",
                        (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        if valid:
            cv2.putText(frame, f"Target: ({int(tx)},{int(ty)}) {dist:.2f}m",
                        (8, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

        try:
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception:
            pass

    def destroy_node(self):
        self.reader_running = False
        self.send_stop()
        time.sleep(0.1)
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
