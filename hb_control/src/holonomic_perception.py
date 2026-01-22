#!/usr/bin/env python3
import math
import cv2
import numpy as np
import traceback
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from hb_interfaces.msg import Pose2D, Poses2D


class PoseDetector(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.homography_ready = False

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.bot_ids = [0, 2, 4]
        self.ref_ids = [1, 3, 5, 7]

        # ---------- MOUSE ----------
        self.mouse_px = None
        self.mouse_py = None

        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05
        self.bots_marker_length = 0.05

        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.crate_poses_pub = self.create_publisher(
            Poses2D, '/crate_pose', 10
        )
        self.bot_poses_pub = self.create_publisher(
            Poses2D, '/bot_pose', 10
        )

        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
            [1030.4890823364258, 0.0, 960.0],
            [0.0, 1030.489103794098, 540.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)

        # ---------- HOMOGRAPHY ----------
        self.pixel_matrix = []
        self.world_matrix = []
        self.H_matrix = None

        self.detected_crates = {}

        # ---------- ARUCO ----------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        # ---------- OPENCV WINDOW (CREATE ONCE) ----------
        cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Detected Markers', 1200, 900)
        cv2.setMouseCallback('Detected Markers', self.mouse_callback)

        self.get_logger().info("PoseDetector initialized")

    # --------------------------------------------------

    def pixel_to_world(self, px, py):
        mat = np.array([[[px, py]]], dtype=np.float32)
        world = cv2.perspectiveTransform(mat, self.H_matrix)
        return world[0][0][0], world[0][0][1]

    # --------------------------------------------------

    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, dist):
        marker_points = np.array([
            [-marker_size/2,  marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)

        rvecs, tvecs, trash = [], [], []
        for c in corners:
            ok, rvec, tvec = cv2.solvePnP(
                marker_points, c, mtx, dist,
                False, cv2.SOLVEPNP_IPPE_SQUARE
            )
            rvecs.append(rvec)
            tvecs.append(tvec)
            trash.append(ok)
        return rvecs, tvecs, trash

    # --------------------------------------------------

    def mouse_callback(self, event, x, y, flags, param):
        if not self.homography_ready:
            return
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_px = x
            self.mouse_py = y

    # --------------------------------------------------

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            undistorted = cv2.undistort(
                img, self.camera_matrix, self.dist_coeffs
            )
            gray = cv2.cvtColor(undistorted, cv2.COLOR_RGB2GRAY)
            gray = cv2.GaussianBlur(gray, (3, 3), 0)

            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is None:
                return

            markers = {ids[i][0]: corners[i][0] for i in range(len(ids))}

            # ---------- HOMOGRAPHY ----------
            if not self.homography_ready:
                required = [1, 3, 5, 7]
                if not all(i in ids.flatten() for i in required):
                    return

                self.pixel_matrix = np.array([
                    markers[1][0],
                    markers[3][1],
                    markers[5][3],
                    markers[7][2],
                ], dtype=np.float32)

                self.world_matrix = np.array([
                    [0, 0],
                    [2438.4, 0],
                    [0, 2438.4],
                    [2438.4, 2438.4]
                ], dtype=np.float32)

                self.H_matrix, _ = cv2.findHomography(
                    self.pixel_matrix, self.world_matrix
                )
                self.homography_ready = True

            rvecs, _, _ = self.my_estimatePoseSingleMarkers(
                corners, self.bots_marker_length,
                self.camera_matrix, self.dist_coeffs
            )

            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            Px, Py = self.pixel_to_world(cx, cy)
            Z_CAM = 2425.0

            cv2.aruco.drawDetectedMarkers(undistorted, corners, ids)

            for idx, mid in enumerate(ids.flatten()):
                if mid in self.ref_ids:
                    continue

                pts = markers[mid]
                cxp = pts[:, 0].mean()
                cyp = pts[:, 1].mean()

                x_obs, y_obs = self.pixel_to_world(cxp, cyp)

                marker_h = 92.25 if mid in {0, 2, 4} else 50.0
                scale = (Z_CAM - marker_h) / Z_CAM

                x_corr = Px + (x_obs - Px) * scale
                y_corr = Py + (y_obs - Py) * scale

                R, _ = cv2.Rodrigues(rvecs[idx])
                yaw = math.atan2(R[1, 0], R[0, 0])

                self.detected_crates[mid] = (x_corr, y_corr, yaw)

                cv2.circle(
                    undistorted,
                    (int(cxp), int(cyp)), 5, (0, 255, 0), -1
                )

            # ---------- MOUSE WORLD DISPLAY ----------
            if self.homography_ready and self.mouse_px is not None:
                wx, wy = self.pixel_to_world(
                    self.mouse_px, self.mouse_py
                )

                cv2.circle(
                    undistorted,
                    (self.mouse_px, self.mouse_py),
                    5, (255, 255, 0), -1
                )

                cv2.putText(
                    undistorted,
                    f"Cursor World X:{wx:.1f}  Y:{wy:.1f}",
                    (self.mouse_px + 10, self.mouse_py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 0), 1
                )

            cv2.imshow('Detected Markers', undistorted)
            cv2.waitKey(1)

            self.publish_crate_poses()
            self.publish_bot_poses()

        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(traceback.format_exc())

    # --------------------------------------------------

    def publish_crate_poses(self):
        msg = Poses2D()
        for mid, (x, y, yaw) in self.detected_crates.items():
            if mid in self.bot_ids or mid in self.ref_ids:
                continue
            p = Pose2D()
            p.id = int(mid)
            p.x = float(x)
            p.y = float(y)
            p.w = float(yaw)
            msg.poses.append(p)
        self.crate_poses_pub.publish(msg)

    def publish_bot_poses(self):
        msg = Poses2D()
        for mid in self.bot_ids:
            if mid in self.detected_crates:
                x, y, yaw = self.detected_crates[mid]
                p = Pose2D()
                p.id = int(mid)
                p.x = float(x)
                p.y = float(y)
                p.w = float(yaw)
                msg.poses.append(p)
        if msg.poses:
            self.bot_poses_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
