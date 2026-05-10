#!/usr/bin/env python3
"""
Two-line lane perception node.

Finds left and right lane lines separately by scanning horizontal slices
of the ROI and splitting white blobs by position relative to image centre.

Debug image: original colour frame with overlaid scan dots, midpoint, and
ROI line. NOT the binary mask.

Subscribes:  image_topic  (sensor_msgs/Image)
Publishes:   error_topic  (std_msgs/Float32)   999.0 = not detected
             debug_topic  (sensor_msgs/Image)
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class LinePerception(Node):

    def __init__(self):
        super().__init__('line_perception')

        self.declare_parameter('image_topic',            '/qcar2/front_camera/image')
        self.declare_parameter('error_topic',            '/line_error')
        self.declare_parameter('debug_topic',            '/qcar2/debug_image')
        self.declare_parameter('hsv_lo',                 [0, 0, 180])
        self.declare_parameter('hsv_hi',                 [179, 60, 255])
        self.declare_parameter('roi_top',                0.5)
        self.declare_parameter('num_scan_rows',          20)
        self.declare_parameter('min_pixels_per_row',     3)
        self.declare_parameter('single_line_offset_frac', 0.3)
        self.declare_parameter('kernel_size',            3)

        p = self.get_parameter
        self._image_topic  = p('image_topic').value
        self._error_topic  = p('error_topic').value
        self._debug_topic  = p('debug_topic').value
        self._hsv_lo       = np.array(p('hsv_lo').value, dtype=np.uint8)
        self._hsv_hi       = np.array(p('hsv_hi').value, dtype=np.uint8)
        self._roi_top      = float(p('roi_top').value)
        self._num_rows     = int(p('num_scan_rows').value)
        self._min_px       = int(p('min_pixels_per_row').value)
        self._offset_frac  = float(p('single_line_offset_frac').value)
        ks                 = int(p('kernel_size').value)
        self._kernel       = cv2.getStructuringElement(cv2.MORPH_RECT, (ks, ks))

        self._bridge    = CvBridge()
        self._error_pub = self.create_publisher(Float32, self._error_topic, 10)
        self._debug_pub = self.create_publisher(Image,   self._debug_topic, 10)
        self.create_subscription(Image, self._image_topic, self._image_cb, 10)

        self.get_logger().info(
            f'Line perception ready  [{self._image_topic}] -> [{self._error_topic}]')

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        h, w   = frame.shape[:2]
        img_cx = w / 2.0
        half_w = img_cx * self._offset_frac

        # ── white mask (used for detection only) ──────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self._hsv_lo, self._hsv_hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        # ── ROI ───────────────────────────────────────────────────────
        roi_y = int(h * self._roi_top)
        roi   = mask[roi_y:, :]
        roi_h = roi.shape[0]

        # ── scan rows ─────────────────────────────────────────────────
        scan_ys  = np.linspace(0, roi_h - 1, self._num_rows, dtype=int)
        left_xs  = []
        right_xs = []
        dbg_pts  = []

        for ry in scan_ys:
            cols = np.where(roi[ry] > 0)[0]
            if len(cols) < self._min_px:
                continue

            y_abs      = roi_y + int(ry)
            left_cols  = cols[cols <  img_cx]
            right_cols = cols[cols >= img_cx]

            if len(left_cols) >= self._min_px:
                lx = float(np.mean(left_cols))
                left_xs.append(lx)
                dbg_pts.append((int(lx), y_abs, (255, 140, 0)))   # orange = left

            if len(right_cols) >= self._min_px:
                rx = float(np.mean(right_cols))
                right_xs.append(rx)
                dbg_pts.append((int(rx), y_abs, (0, 80, 255)))    # blue = right

        # ── compute midpoint ──────────────────────────────────────────
        have_left  = len(left_xs)  > 0
        have_right = len(right_xs) > 0

        if have_left and have_right:
            mid_x  = (np.mean(left_xs) + np.mean(right_xs)) / 2.0
            status = 'BOTH'
        elif have_left:
            mid_x  = np.mean(left_xs) + half_w
            status = 'LEFT-ONLY'
        elif have_right:
            mid_x  = np.mean(right_xs) - half_w
            status = 'RIGHT-ONLY'
        else:
            mid_x  = None
            status = 'SEARCHING'

        # ── error ─────────────────────────────────────────────────────
        if mid_x is not None:
            error = float(np.clip((mid_x - img_cx) / img_cx, -1.0, 1.0))
        else:
            error = math.nan

        out      = Float32()
        out.data = 999.0 if math.isnan(error) else error
        self._error_pub.publish(out)

        # ── debug image: draw ON THE ORIGINAL COLOUR FRAME ────────────
        dbg = frame.copy()   # <-- colour, not the mask

        # semi-transparent ROI shading
        overlay        = dbg.copy()
        overlay[roi_y:] = (overlay[roi_y:].astype(np.int32) + [0, 0, 40]).clip(0, 255).astype(np.uint8)
        cv2.addWeighted(overlay, 0.4, dbg, 0.6, 0, dbg)

        # ROI boundary line
        cv2.line(dbg, (0, roi_y), (w, roi_y), (255, 220, 0), 2)

        # image centre line (dashed)
        for y in range(roi_y, h, 12):
            cv2.line(dbg, (int(img_cx), y), (int(img_cx), min(y + 6, h)),
                     (180, 180, 180), 1)

        # scan point dots
        for px, py, col in dbg_pts:
            cv2.circle(dbg, (px, py), 5, col, -1)
            cv2.circle(dbg, (px, py), 5, (255, 255, 255), 1)   # white ring

        # midpoint indicator
        if mid_x is not None:
            mid_y = roi_y + roi_h // 2
            cv2.circle(dbg, (int(mid_x), mid_y), 10, (0, 255, 80), -1)
            cv2.circle(dbg, (int(mid_x), mid_y), 10, (255, 255, 255), 2)
            label = f'{status}  err={error:+.3f}'
        else:
            label = status

        # status text with dark background for readability
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        cv2.rectangle(dbg, (8, 6), (14 + tw, 14 + th), (0, 0, 0), -1)
        cv2.putText(dbg, label, (10, 10 + th),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        self._debug_pub.publish(
            self._bridge.cv2_to_imgmsg(dbg, encoding='rgb8'))


def main():
    rclpy.init()
    node = LinePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()