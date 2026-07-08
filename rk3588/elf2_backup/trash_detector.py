#!/usr/bin/env python3
"""
Red Cylinder Detector with Tracking, Trajectory Prediction, and STM32 Serial Output
===================================================================================

Pipeline:
  Camera -> Red cylinder detection -> Multi-frame Kalman tracking ->
  Velocity estimation -> 3D trajectory prediction -> Binary frame -> STM32F103

Architecture:
  RK3588 (this node) --UART(/dev/ttyS9)--> STM32F103C8T6 --> 2x Motors

New features:
  1. 垃圾类别 — 是否是需要接的垃圾 (category flag in binary frame)
  2. 像素坐标 u, v — 目标在图像中的位置
  3. 运动速度 dx/dt, dy/dt — 像素空间速度 (Kalman 滤波)
  4. 落点坐标 x_mm, y_mm — 世界坐标，相对垃圾桶中心
  5. 到达时间 arrive_ms — 抛物线运动模型预测
  6. 二进制帧 UART 输出 — 10Hz, 18-byte protocol

Binary Frame Protocol (18 bytes, little-endian):
  Byte 0-1:   Header 0xAA 0x55
  Byte 2:     Flags (bit0=垃圾类别, bit1=检测有效)
  Byte 3-4:   u       uint16  像素 X 坐标
  Byte 5-6:   v       uint16  像素 Y 坐标
  Byte 7-8:   dx      int16   像素速度 X (*100)
  Byte 9-10:  dy      int16   像素速度 Y (*100)
  Byte 11-12: x_mm    int16   预测落点 X (mm, 相对垃圾桶)
  Byte 13-14: y_mm    int16   预测落点 Y (mm, 相对垃圾桶)
  Byte 15-16: arrive_ms uint16 预计到达时间 (ms)
  Byte 17:    checksum       XOR of bytes [2..16]

Dependencies:
  pip3 install pyserial
  sudo chmod 666 /dev/ttyS9   (or: sudo usermod -aG dialout $USER)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2, numpy as np
import serial
import threading
import struct
import time


# ============================================================
# Binary Frame Protocol (18 bytes → STM32)
# ============================================================
class BinaryFrame:
    HEADER = bytes([0xAA, 0x55])

    def __init__(self):
        self.category   = 0   # 0=ignore, 1=target garbage to catch
        self.valid      = 0   # 0=no target, 1=target detected
        self.u          = 0   # pixel x
        self.v          = 0   # pixel y
        self.dx         = 0   # pixel velocity x (*100)
        self.dy         = 0   # pixel velocity y (*100)
        self.x_mm       = 0   # predicted landing x (mm, rel to bin)
        self.y_mm       = 0   # predicted landing y (mm, rel to bin)
        self.arrive_ms  = 0   # time to arrival (ms)

    def pack(self) -> bytes:
        """STM32 CMD_SET_GOAL (0x02): AA 55 02 06 x_mm y_mm arrive_ms checksum"""
        payload  = struct.pack('<h',  self.x_mm)
        payload += struct.pack('<h',  self.y_mm)
        payload += struct.pack('<H',  self.arrive_ms)
        cmd_type = 0x02  # CMD_SET_GOAL
        length = len(payload)  # 6
        checksum = (cmd_type + length) & 0xFF
        for b in payload:
            checksum = (checksum + b) & 0xFF
        return self.HEADER + bytes([cmd_type, length]) + payload + bytes([checksum])

    def __repr__(self):
        return (f"BF(cat={self.category} v={self.valid} "
                f"uv=({self.u},{self.v}) vel=({self.dx},{self.dy}) "
                f"land=({self.x_mm},{self.y_mm})mm T={self.arrive_ms}ms)")


# ============================================================
# Kalman Filter Tracker (6-state constant velocity model)
# State: [u, v, d, du, dv, dd]
#   u,v = pixel position
#   d   = inverse depth (1/distance)
#   du,dv,dd = velocities
# ============================================================
class KalmanTracker:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.kf = cv2.KalmanFilter(6, 3, 0)

        self.kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0,  0 ],
            [0, 1, 0, 0,  dt, 0 ],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0 ],
            [0, 0, 0, 0,  1,  0 ],
            [0, 0, 0, 0,  0,  1 ]
        ], np.float32)

        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], np.float32)

        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-2
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 5e-2
        self.kf.errorCovPost = np.eye(6, dtype=np.float32)

        self.initialized = False
        self.confidence  = 0.0

    def update(self, u, v, d):
        meas = np.array([[np.float32(u)],
                         [np.float32(v)],
                         [np.float32(d)]])
        if not self.initialized:
            self.kf.statePost = np.array([[u],[v],[d],[0.0],[0.0],[0.0]], np.float32)
            self.initialized = True
            self.confidence = 0.3
        else:
            self.kf.predict()
            self.kf.correct(meas)
            self.confidence = min(1.0, self.confidence + 0.08)
        return self.get_state()

    def predict_only(self):
        if self.initialized:
            self.kf.predict()
            self.confidence = max(0.0, self.confidence - 0.15)
        return self.get_state()

    def get_state(self):
        return self.kf.statePost.flatten()

    def get_velocity_pixels(self):
        s = self.kf.statePost.flatten()
        return s[3], s[4]


# ============================================================
# Main Detector Node
# ============================================================
class TrashDetector(Node):
    def __init__(self):
        super().__init__("trash_detector")
        self.bridge = CvBridge()

        # ---------- parameters ----------
        self.declare_parameter("camera_ns", "ascamera_hp60c")
        ns = self.get_parameter("camera_ns").value

        # camera intrinsics (tune for your camera!)
        self.declare_parameter("fx", 450.0)
        self.declare_parameter("fy", 450.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        # camera extrinsics (relative to bin frame)
        self.declare_parameter("cam_height_mm", 400.0)
        self.declare_parameter("cam_forward_mm", 100.0)
        self.declare_parameter("cam_pitch_deg", -15.0)  # negative = looking down

        # object properties
        self.declare_parameter("object_real_diameter_mm", 66.0)
        self.declare_parameter("garbage_category", 1)   # 1 = red cylinder is target

        # serial
        self.declare_parameter("serial_port", "/dev/ttyS9")
        self.declare_parameter("serial_baud", 115200)

        # ---------- subscriptions ----------
        self.sub = self.create_subscription(Image,
            f"/{ns}/camera_publisher/rgb0/image", self.callback,
            qos_profile_sensor_data)

        # ---------- publishers ----------
        self.pub_result = self.create_publisher(Image, "/trash_detector/result", 10)
        self.pub_target = self.create_publisher(Float32MultiArray, "/trash_detector/target", 10)

        # ---------- tracker ----------
        self.tracker = KalmanTracker(dt=0.1)
        self.prev_time = self.get_clock().now()
        self.track_lost_count = 0

        # ---------- serial ----------
        self.serial_lock = threading.Lock()
        self.latest_frame = BinaryFrame()
        self.serial_running = True

        try:
            self.ser = serial.Serial(
                port=self.get_parameter("serial_port").value,
                baudrate=self.get_parameter("serial_baud").value,
                timeout=0.1)
            self.get_logger().info(f"Serial open: {self.ser.name}")
        except Exception as e:
            self.get_logger().warn(f"Serial failed (will retry): {e}")
            self.ser = None

        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.serial_thread.start()

        self.fc = 0
        self.get_logger().info("Trash Detector ready — tracking + trajectory + STM32 out")

    # ----------------------------------------------------------------
    #  Serial send thread (10 Hz)
    # ----------------------------------------------------------------
    def _serial_loop(self):
        while self.serial_running and rclpy.ok():
            if self.ser is not None:
                try:
                    with self.serial_lock:
                        data = self.latest_frame.pack()
                    self.ser.write(data)
                    self.ser.flush()
                except Exception as e:
                    self.get_logger().warn(f"Serial write: {e}")
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
            else:
                try:
                    self.ser = serial.Serial(
                        port=self.get_parameter("serial_port").value,
                        baudrate=self.get_parameter("serial_baud").value,
                        timeout=0.1)
                    self.get_logger().info(f"Serial reconnected: {self.ser.name}")
                except Exception:
                    pass
            time.sleep(0.1)   # 10Hz

    # ----------------------------------------------------------------
    #  pixel → world (bin frame) conversion
    # ----------------------------------------------------------------
    def _pixel_to_world(self, u, v, pixel_diameter):
        """Convert pixel coords + apparent diameter → world coords (mm) in bin frame.
        Returns (x_mm, y_mm, z_mm)."""
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        real_d  = self.get_parameter("object_real_diameter_mm").value
        cam_h   = self.get_parameter("cam_height_mm").value
        cam_fwd = self.get_parameter("cam_forward_mm").value
        pitch   = np.radians(self.get_parameter("cam_pitch_deg").value)

        if pixel_diameter < 1:
            pixel_diameter = 1.0

        # depth from known object size
        Z_cam = fx * real_d / pixel_diameter                    # mm

        # camera-frame coordinates
        X_cam = (u - cx) * Z_cam / fx                           # mm lateral
        Y_cam = -(v - cy) * Z_cam / fy                          # mm vertical (negate pixel y)

        # rotate by camera pitch around X axis
        ang = -pitch
        cos_p, sin_p = np.cos(ang), np.sin(ang)
        Y_world = Y_cam * cos_p - Z_cam * sin_p                 # forward
        Z_world = Y_cam * sin_p + Z_cam * cos_p                 # upward
        X_world = X_cam

        # offset to bin origin (on ground)
        x_bin = X_world
        y_bin = Y_world + cam_fwd
        z_bin = cam_h - Z_world

        return x_bin, y_bin, z_bin

    # ----------------------------------------------------------------
    #  landing prediction (projectile motion under gravity)
    # ----------------------------------------------------------------
    @staticmethod
    def _predict_landing(x, y, z, vx, vy, vz):
        """Predict where object hits z=0 plane.

        Returns (x_land_mm, y_land_mm, arrive_ms)."""
        g = 9810.0                                              # mm/s²

        if z <= 0:
            return x, y, 0.0

        # solve: z + vz*t - 0.5*g*t² = 0  →  (g/2)·t² - vz·t - z = 0
        a = 0.5 * g
        b = -vz
        c = -z
        disc = b * b - 4.0 * a * c
        if disc < 0 or a < 1e-6:
            t = np.sqrt(2.0 * z / g)
        else:
            sq = np.sqrt(disc)
            t1 = (-b + sq) / (2.0 * a)
            t2 = (-b - sq) / (2.0 * a)
            candidates = [t for t in (t1, t2) if t > 0.001]
            t = min(candidates) if candidates else np.sqrt(2.0 * z / g)

        x_land = x + vx * t
        y_land = y + vy * t
        arrive_ms = min(t * 1000.0, 65535.0)
        return x_land, y_land, arrive_ms

    # ----------------------------------------------------------------
    #  main callback
    # ----------------------------------------------------------------
    def callback(self, msg):
        self.fc += 1
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:
            dt = 0.1
        self.prev_time = now
        self.tracker.dt = dt

        # --- decode ---
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        h, w = frame.shape[:2]

        # --- half-res for speed ---
        small = cv2.resize(frame, (w // 2, h // 2))
        fx, fy = 2.0, 2.0

        # --- red color mask (dual HSV) ---
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 150, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 150, 80), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # --- shape analysis ---
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 100 or area > 30000:
                continue
            peri = cv2.arcLength(c, True)
            if peri == 0:
                continue
            circ = 4.0 * np.pi * area / (peri * peri)
            rx, ry, rbw, rbh = cv2.boundingRect(c)
            rect_fill = area / (rbw * rbh) if rbw * rbh > 0 else 0
            ar = max(rbw, rbh) / min(rbw, rbh) if min(rbw, rbh) > 0 else 1
            if circ > 0.55:
                valid.append((c, area, circ, 'top', ar, rect_fill))
            elif rect_fill > 0.65 and 1.3 < ar < 5.0:
                valid.append((c, area, circ, 'side', ar, rect_fill))

        # --- Hough circles ---
        gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray_small, gray_small, mask=mask)
        blurred = cv2.GaussianBlur(masked_gray, (9, 9), 2)
        hcircles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
                                     dp=1.2, minDist=30, param1=100, param2=25,
                                     minRadius=5, maxRadius=80)

        # --- draw crosshair ---
        cv2.line(frame, (w//2-15, h//2), (w//2+15, h//2), (0,255,255), 1)
        cv2.line(frame, (w//2, h//2-15), (w//2, h//2+15), (0,255,255), 1)

        if hcircles is not None:
            hcircles = np.uint16(np.around(hcircles))
            for c in hcircles[0, :]:
                cv2.circle(frame, (c[0]*2, c[1]*2), c[2]*2, (0,255,255), 1)

        # --- draw targets & find best ---
        best_target = None
        best_area = 0.0

        for i, (c, area, circ, vtype, ar, rf) in enumerate(sorted(valid, key=lambda x: -x[1])):
            x, y, bw, bh = cv2.boundingRect(c)
            x, y, bw, bh = int(x*fx), int(y*fy), int(bw*fx), int(bh*fy)
            cx, cy = x + bw//2, y + bh//2
            est = 1.0 / (np.sqrt(area*4)/35.0) if area > 0 else 0

            if vtype == 'top':
                color = (0,255,0); label = "TOP %.1fm" % est
            else:
                color = (255,0,255); label = "SIDE %.1fm" % est

            cv2.rectangle(frame, (x,y), (x+bw, y+bh), color, 2)
            (ecx, ecy), er = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(ecx*fx), int(ecy*fy)), int(er*fx), color, 1)
            cv2.putText(frame, "#%d %s"%(i+1, label), (x, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            if area > best_area:
                best_area = area
                pixel_diameter = np.sqrt(4.0*area/np.pi) * fx
                best_target = [float(cx), float(cy), float(est),
                               float(area), pixel_diameter]

        # ================================================================
        #  Tracking + Velocity + Trajectory + Serial
        # ================================================================
        bf = BinaryFrame()
        bf.category = self.get_parameter("garbage_category").value

        if best_target is not None:
            cx, cy, dist_m, area_small, pixel_diameter = best_target

            # Kalman update
            d = 1.0 / max(dist_m, 0.01)
            state = self.tracker.update(cx, cy, d)
            self.track_lost_count = 0

            u_filt, v_filt, d_filt = state[0], state[1], state[2]
            du_pix, dv_pix = self.tracker.get_velocity_pixels()

            # world velocity
            dist_filt = 1.0 / max(d_filt, 0.001)
            fx_cam = self.get_parameter("fx").value
            fy_cam = self.get_parameter("fy").value
            vx_mm_s = du_pix * dist_filt / fx_cam
            vy_mm_s = dv_pix * dist_filt / fy_cam
            dd = state[5]
            vz_mm_s = -dd / (d_filt * d_filt) if d_filt > 0.001 else 0.0

            # world position
            x_w, y_w, z_w = self._pixel_to_world(u_filt, v_filt, pixel_diameter)

            # landing prediction
            x_land, y_land, arrive_ms = self._predict_landing(
                x_w, y_w, z_w, vx_mm_s, vy_mm_s, vz_mm_s)

            # fill binary frame
            bf.valid = 1
            bf.u     = int(np.clip(u_filt, 0, 65535))
            bf.v     = int(np.clip(v_filt, 0, 65535))
            bf.dx    = int(np.clip(du_pix * 100.0, -32768, 32767))
            bf.dy    = int(np.clip(dv_pix * 100.0, -32768, 32767))
            bf.x_mm  = int(np.clip(x_land, -32768, 32767))
            bf.y_mm  = int(np.clip(y_land, -32768, 32767))
            bf.arrive_ms = int(arrive_ms)

            # ROS publish (backward compatible)
            msg_out = Float32MultiArray()
            msg_out.data = [float(cx), float(cy), float(dist_m), float(area_small)]
            self.pub_target.publish(msg_out)

            # --- draw prediction info ---
            cv2.putText(frame,
                f"UV:({bf.u},{bf.v}) V:({bf.dx/100:.0f},{bf.dy/100:.0f})px/s",
                (8, h-80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,255,255), 1)
            cv2.putText(frame,
                f"World:({x_w:.0f},{y_w:.0f},{z_w:.0f})mm Vz:{vz_mm_s:.0f}mm/s",
                (8, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,255,255), 1)
            cv2.putText(frame,
                f"LAND:({bf.x_mm},{bf.y_mm})mm | {bf.arrive_ms}ms",
                (8, h-40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,200), 1)

            # landing prediction marker
            pred_x = int(np.clip(u_filt + du_pix * arrive_ms / 1000.0, 0, w-1))
            pred_y = int(np.clip(v_filt + dv_pix * arrive_ms / 1000.0, 0, h-1))
            cv2.drawMarker(frame, (pred_x, pred_y), (0,0,255),
                           cv2.MARKER_DIAMOND, 15, 2)

        else:
            self.track_lost_count += 1
            if self.track_lost_count <= 5:
                state = self.tracker.predict_only()
                if self.tracker.confidence > 0.1:
                    bf.valid = 1
                    bf.u  = int(np.clip(state[0], 0, 65535))
                    bf.v  = int(np.clip(state[1], 0, 65535))
                    bf.dx = int(np.clip(state[3]*100.0, -32768, 32767))
                    bf.dy = int(np.clip(state[4]*100.0, -32768, 32767))
            else:
                bf.valid = 0

        # push to serial thread
        with self.serial_lock:
            self.latest_frame = bf

        # --- status bar ---
        sc = (0,255,0) if bf.valid else (0,0,255)
        cv2.putText(frame,
            f"SER:{'OK' if self.ser else 'NO'} TRACK:{self.tracker.confidence:.1f}",
            (8, h-18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, sc, 1)
        cv2.putText(frame, f"CYL:{len(valid)} F:{self.fc}", (8,22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)

        try:
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception:
            pass

    # ----------------------------------------------------------------
    #  cleanup
    # ----------------------------------------------------------------
    def destroy_node(self):
        self.serial_running = False
        if hasattr(self, 'serial_thread'):
            self.serial_thread.join(timeout=1.0)
        if hasattr(self, 'ser') and self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
