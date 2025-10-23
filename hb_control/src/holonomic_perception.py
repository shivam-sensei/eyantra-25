#!/usr/bin/env python3
"""
This Python file runs a ROS 2 node named localization_node which publishes the position of crates and a holonomic drive robot.
This node subscribes to the following topics:
 SUBSCRIPTIONS
 /camera/image_raw
 /camera/camera_info
 /crates_pose
 /bot_pose
"""
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
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        self.processing = False
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = 'DICT_4X4_50'  # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
            [1030.4890823364258,0.0,960.0],
            [0.0,1030.489103794098,540.0],
            [0.0,0.0,1.0]
        ])  # load camera intrinsics (3x3 matrix)
        self.dist_coeffs = np.array([0.0,0.0,0.0,0.0,0.0])    # load distortion coefficients (1x5 array)
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = []  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = []  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
        self.detected_crates = {}
        
        # ---------- ARUCO SETUP ----------
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):
        """
        - Calculate the H_matrix using: use cv2.findHomography
        - Convert the pixel coordinates into real world coordinates using: cv2.perspectiveTransform(src_pts, self.H_matrix)
        """
        matlike_pixel =np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
        world_points = cv2.perspectiveTransform(matlike_pixel,self.H_matrix)
        # Implement pixel to world coordinate conversion
        # Step 1: Ensure H_matrix is computed
        # Step 2: Create pixel point in correct format for cv2.perspectiveTransform
        # Step 3: Apply transformation and return world coordinates
        return world_points[0][0][0], world_points[0][0][1]
    
    def my_estimatePoseSingleMarkers(self,corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def image_callback(self, msg):
        if self.processing:
            return
        
        self.processing = True
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            undistorted_img = cv2.undistort(img, self.camera_matrix, self.dist_coeffs, None, None)

            gray = cv2.cvtColor(undistorted_img, cv2.COLOR_RGB2GRAY)

            # self.get_logger().info("Step 4: Applying CLAHE")
            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            # enhanced = clahe.apply(gray)

            # self.get_logger().info("Step 5: Applying Gaussian blur")
            # gray_processed = cv2.GaussianBlur(enhanced, (3, 3), 0)

            corners, ids, rejected = self.detector.detectMarkers(gray)

            if ids is None or len(ids) == 0:
                self.get_logger().warn("No markers detected")
                self.processing = False
                return

            markers = {ids[i][0]: corners[i][0] for i in range(len(ids))}

            required_ids = [1, 3, 5, 7]
            if not all(i in ids.flatten().tolist() for i in required_ids):
                self.get_logger().warn("Missing reference markers (1,3,5,7). Skipping frame.")
                self.processing = False
                return

            self.pixel_matrix = np.array([
                markers[1][0],  
                markers[3][1],
                markers[5][3],
                markers[7][2],
            ], dtype=np.float32)
            self.get_logger().info(f"Pixel matrix: {self.pixel_matrix}")

            self.world_matrix = np.array([
                [0, 0],
                [2438.4, 0],
                [0, 2438.4],
                [2438.4, 2438.4]
            ], dtype=np.float32)

            self.H_matrix, _ = cv2.findHomography(self.pixel_matrix, self.world_matrix)
            if self.H_matrix is None:
                raise ValueError("Homography matrix computation failed")

            r, t, _ = self.my_estimatePoseSingleMarkers(
                corners, self.bots_marker_length, self.camera_matrix, self.dist_coeffs
            )

            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            # map principal point to world (mm)
            Px, Py = self.pixel_to_world(cx, cy)

            Z_CAM = 2400.0  # camera height in mm

            for idx, marker_id in enumerate(ids.flatten()):
                if marker_id in [1, 3, 5, 7]:
                    continue

                pts = markers[marker_id]
                centre_x = pts[:, 0].mean()
                centre_y = pts[:, 1].mean()

                # observed map position (mm) from homography
                x_obs, y_obs = self.pixel_to_world(centre_x, centre_y)

                # choose marker height (in mm). Example: bot id 0 -> 95.25 mm, others maybe 0
                if marker_id == 0:
                    marker_h = 95.25
                else:
                    marker_h = 50.0  # change if you know other heights

                Z_marker = Z_CAM - marker_h
                scale = Z_marker / Z_CAM  # e.g. ~0.96 for 95.25 mm

                # correct about the principal point mapped to world
                x_corr = Px + (x_obs - Px) * scale
                y_corr = Py + (y_obs - Py) * scale

                # compute yaw from rvecs as before
                rvec = r[idx]
                R_mat, _ = cv2.Rodrigues(rvec)
                yaw_rad = math.atan2(R_mat[1, 0], R_mat[0, 0])
                
                self.detected_crates[marker_id] = (x_corr, y_corr, yaw_rad)

            # cv2.namedWindow('Detected Markers', cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('Detected Markers', 1200, 900)
            # cv2.imshow('Detected Markers', undistorted_img)
            # cv2.waitKey(1)

            self.publish_crate_poses()
            self.publish_bot_poses()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            self.get_logger().error(traceback.format_exc())

        finally:
            self.processing = False


    def publish_crate_poses(self):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.crate_poses_pub.publish(msg)
        """
        poses_msg = Poses2D()
        # Publish all detected crates
        for detected_id, (x, y, yaw) in self.detected_crates.items():
            if detected_id != 0:
                crate_pose = Pose2D()
                crate_pose.id = int(detected_id)
                crate_pose.x = float(x)
                crate_pose.y = float(y)
                crate_pose.w = float(yaw)
                poses_msg.poses.append(crate_pose)
            
        self.crate_poses_pub.publish(poses_msg)
        pass

    def publish_bot_poses(self):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.bot_poses_pub.publish(msg)
        """
        if 0 in self.detected_crates:
            bot_pose_msg = Poses2D()
            bot_pose = Pose2D()
            bot_pose.id = 0
            bot_pose.x = float(self.detected_crates[0][0])
            bot_pose.y = float(self.detected_crates[0][1])
            bot_pose.w = float(self.detected_crates[0][2])
            bot_pose_msg.poses.append(bot_pose)
            self.bot_poses_pub.publish(bot_pose_msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    pose_detector = PoseDetector()
    try:
        rclpy.spin(pose_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()