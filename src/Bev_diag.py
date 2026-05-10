#!/usr/bin/env python3
"""
BEV diagnostic — subscribe to the debug image and print where white
pixels actually are in the BEV canvas. Run this, drive the car one
full lap, and paste the output here.

Usage:
  ros2 run your_package bev_diagnostic
  OR
  python3 bev_diagnostic.py
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


BIRDW        = 800
BIRDH        = 600
BEV_M_PER_PX = 0.002

H_CAM  = 0.176
HFOV   = 1.5708
IMG_W  = 640
IMG_H  = 480
FX     = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
FY     = FX
CX_IMG = IMG_W / 2.0
CY_IMG = IMG_H / 2.0


def _ground_to_img(d, s):
    u = FX * s / d + CX_IMG
    v = FY * H_CAM / d + CY_IMG
    return float(u), float(v)


def _ground_to_bev(d, s):
    col = BIRDW / 2.0 + s / BEV_M_PER_PX
    row = BIRDH / 2.0 - d / BEV_M_PER_PX
    return float(col), float(row)


def _build_homography():
    pts = [(0.20, -0.40), (0.20, +0.40), (0.70, -0.40), (0.70, +0.40)]
    src = np.float32([_ground_to_img(d, s) for d, s in pts])
    dst = np.float32([_ground_to_bev(d, s) for d, s in pts])
    H, _ = cv2.findHomography(src, dst)
    return H

_H_BEV = _build_homography()


class BEVDiagnostic(Node):
    def __init__(self):
        super().__init__('bev_diagnostic')
        self._bridge  = CvBridge()
        self._count   = 0
        self.create_subscription(
            Image, '/qcar2/front_camera/image', self._cb, 10)
        self.get_logger().info('BEV diagnostic running — watching camera...')

    def _cb(self, msg):
        self._count += 1
        if self._count % 15 != 0:   # sample every 15 frames
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        bev   = cv2.warpPerspective(frame, _H_BEV, (BIRDW, BIRDH))
        hsv   = cv2.cvtColor(bev, cv2.COLOR_RGB2HSV)

        # White mask
        white = cv2.inRange(hsv,
                            np.array([0,   0, 190], dtype=np.uint8),
                            np.array([179, 40, 255], dtype=np.uint8))

        # Orange mask
        orange = cv2.inRange(hsv,
                             np.array([5,  100, 100], dtype=np.uint8),
                             np.array([25, 255, 255], dtype=np.uint8))

        self.get_logger().info('─' * 60)
        self.get_logger().info(f'Frame {self._count}')

        # For each of 5 evenly spaced rows in 50..450, report white clusters
        for row in [100, 150, 200, 250, 300, 350]:
            w_cols = np.where(white[row] > 0)[0]
            o_cols = np.where(orange[row] > 0)[0]

            # cluster white
            w_clusters = self._cluster(w_cols)
            o_clusters = self._cluster(o_cols)

            w_str = '  '.join(
                f'col={int(cx):3d}(w={w}px,{w*BEV_M_PER_PX*100:.1f}cm)'
                for cx, w in w_clusters
            ) or 'none'
            o_str = '  '.join(
                f'col={int(cx):3d}(w={w}px)'
                for cx, w in o_clusters
            ) or 'none'

            self.get_logger().info(
                f'  row={row:3d}  WHITE:[{w_str}]  ORANGE:[{o_str}]'
            )

        # Also print the overall bounding box of all white pixels
        ys, xs = np.where(white > 0)
        if len(ys):
            self.get_logger().info(
                f'  WHITE pixel extent: rows {ys.min()}..{ys.max()}  '
                f'cols {xs.min()}..{xs.max()}'
            )
        else:
            self.get_logger().info('  NO white pixels found in BEV!')

        ys, xs = np.where(orange > 0)
        if len(ys):
            self.get_logger().info(
                f'  ORANGE pixel extent: rows {ys.min()}..{ys.max()}  '
                f'cols {xs.min()}..{xs.max()}'
            )
        else:
            self.get_logger().info('  NO orange pixels found in BEV!')

        # Save a debug BEV image to /tmp so you can view it
        dbg = bev.copy()
        dbg[white  > 0] = [255, 255,   0]   # yellow = white pixels
        dbg[orange > 0] = [255, 128,   0]   # orange
        # draw row lines
        for row in [100, 150, 200, 250, 300, 350]:
            cv2.line(dbg, (0, row), (BIRDW-1, row), (0, 255, 0), 1)
        cv2.line(dbg, (BIRDW//2, 0), (BIRDW//2, BIRDH-1), (0,200,200), 1)
        cv2.imwrite(f'/tmp/bev_diag_{self._count:04d}.png', 
                    cv2.cvtColor(dbg, cv2.COLOR_RGB2BGR))
        self.get_logger().info(
            f'  Saved /tmp/bev_diag_{self._count:04d}.png')


    def _cluster(self, cols, gap=15):
        if len(cols) == 0:
            return []
        result = []
        start = prev = int(cols[0])
        for c in cols[1:]:
            c = int(c)
            if c - prev > gap:
                result.append(((start+prev)/2.0, prev-start+1))
                start = c
            prev = c
        result.append(((start+prev)/2.0, prev-start+1))
        return [(cx, w) for cx, w in result if w >= 3]


def main():
    rclpy.init()
    node = BEVDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()s