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
from matplotlib.pyplot import gray
import numpy as np
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
        
        # ---------- PARAMETERS ----------
        self.crates_marker_length = 0.05  # Set marker size in meters
        self.bots_marker_length = 0.05    # Set bot marker size in meters
        self.aruco_dict_name = 'DICT_4X4_50'  # Choose ArUco dictionary
        
        # ---------- TOPICS ----------
        # self.image_sub = self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.crate_poses_pub = self.create_publisher(Poses2D, '/crate_pose', 10)
        self.bot_poses_pub = self.create_publisher(Poses2D, '/bot_pose', 10)
        
        # ---------- CAMERA PARAMETERS ----------
        self.camera_matrix = np.array([
                [1410.750171 ,0.000000 ,804.027477],
                [0.000000 ,1405.828447, 535.979552],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)

        self.dist_coeffs = np.array([0.038664 ,-0.088372 ,-0.006480 ,-0.024568 ,0.000000]) 
        
        # ---------- IMAGE MATRICES ----------
        self.pixel_matrix = [[446.0, 27.0],[1474.0,26.0],[445.0,1055.0],[1475.0, 1055.0]]  # derive pixel points matrix [[x1,y1], [x2,y2], ...]
        self.world_matrix = [[0,0],[2438.4, 0],[0, 2438.4],[2438.4, 2438.4]]  # derive world points matrix [[x1,y1], [x2,y2], ...]
        self.H_matrix = None    # compute homography matrix using cv2.findHomography
     
 
        self.H_matrix = None   
        

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()



        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 35
        self.aruco_params.adaptiveThreshWinSizeStep = 4 
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.02



        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                

        self.x_real_1 = np.array([1218.84,298.2,498.2, 1219.2, 1219.2,819.2,869.2]) 
        self.x_est_1 = np.array([1219.2,274.46,478.98, 1218.86, 1218.85, 808.45,864.94]) 
        self.y_real_1 = np.array([205.0, 1031.2, 831.2, 1219.2, 719.2, 1019.2,]) 
        self.y_est_1 = np.array([169.4, 1027.31, 821.49, 1219.41, 707.28, 1014.27,])
        self.a_x_1 , self.b_x_1 = np.polyfit(self.x_est_1,self.x_real_1,1)
        self.a_y_1 , self.b_y_1 = np.polyfit(self.y_est_1,self.y_real_1,1)

        self.x_real_2 = np.array([2152.2,1582.11,1951.2,1914.2,1912.0]) 
        self.x_est_2 = np.array([2174.66,1569.2,1958.97,1924.2,1921.2]) 
        self.y_real_2 = np.array([724.2,219.2,667.2,940.0,1019.0]) 
        self.y_est_2 = np.array([711.35,169.37,660.18,925,1040.0])
        self.a_x_2 , self.b_x_2 = np.polyfit(self.x_est_2,self.x_real_2,1)
        self.a_y_2 , self.b_y_2 = np.polyfit(self.y_est_2,self.y_real_2,1)

        self.cap = cv2.VideoCapture(2)

        
        self.timer = self.create_timer(0.03, self.image_callback)  # ~30 FPS

        self.get_logger().info('PoseDetector initialized')

    def pixel_to_world(self, pixel_x, pixel_y):

        if self.H_matrix is None:
            print("Error: Homography matrix not computed yet!")
            return None, None

       
        pixel_pt = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)

        
        world_pt = cv2.perspectiveTransform(pixel_pt, self.H_matrix)  

  
        x_world, y_world = world_pt[0][0]

        if (x_world>=0 and x_world<=1219.2) and (y_world>=0 and y_world<=1219.2):
            
            x_world = self.a_x_1*x_world + self.b_x_1
            y_world = self.a_y_1*y_world + self.b_y_1

        if (x_world>1219.2 and x_world<=2438.4) and (y_world>=0 and y_world<=1219.2):
            
            x_world = self.a_x_2*x_world + self.b_x_2
            y_world = self.a_y_2*y_world + self.b_y_2

        return x_world, y_world


    def image_callback(self):

        if not self.cap.isOpened():
            self.get_logger().error("Camera not opened")
            return

        ret, frame = self.cap.read()
        if not ret:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.apply_gamma(gray, gamma=1.8)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        crate_poses = {}
        bot_poses = {}


        cv2.aruco.drawDetectedMarkers(gray, corners, ids)
            # dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            # cv2.aruco.drawDetectedMarkers(dbg, rejected, borderColor=(0,0,255))
            # cv2.imshow("Rejected Candidates", dbg)

            

        cv2.imshow("Gray Image", gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def apply_gamma(self, gray, gamma=1.2):
        inv_gamma = 1.0 / gamma
        table = np.array(
            [((i / 255.0) ** inv_gamma) * 255 for i in range(256)],
            dtype="uint8"
        )
        return cv2.LUT(gray, table)


    def publish_crate_poses(self, poses):

        msg = Poses2D()
        
        for marker_id, (x, y, w) in poses.items():
            
            pose = Pose2D()
            pose.id = int(marker_id)
           
            pose.x = float(x)
            pose.y = float(y)
            pose.w = float(w)
            msg.poses.append(pose)
        self.crate_poses_pub.publish(msg)
        pass

    def publish_bot_poses(self, poses):
        """
        - Convert python pose dictionary -> message (Poses2D)
        - self.bot_poses_pub.publish(msg)
        """
        # Create Poses2D message
        # For each pose in poses list:
        #       - Create Pose2D message
        #       - Set id, x, y, w fields
        #       - Append to poses message
        # Publish the message

        msg = Poses2D()
        for marker_id, (x, y, w) in poses.items():
            pose = Pose2D()
            pose.id = int(marker_id)
            pose.x = float(x)
            pose.y = float(y)
            pose.w = float(w)
            msg.poses.append(pose)
        self.bot_poses_pub.publish(msg)

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