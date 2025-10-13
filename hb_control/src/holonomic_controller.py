#!/usr/bin/env python3
# ---------------------- Import Required Libraries ----------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# import hb_interface messages
import numpy as np
import math
from hb_interfaces.msg import Pose2D, Poses2D


# ---------------------- PID Controller Class --------------------------------
class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        kd_error = (error - self.prev_error)/dt
        value = error*self.kp + kd_error*self.kd
        self.prev_error = error
        return value
    
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node

        # ---------------- Robot Parameters ----------------
        self.robot_id = 9
        self.current_pose = None
        self.goal_index = 0
        self.last_time = self.get_clock().now()
        self.goal_limit = {
            'x': 2.5,
            'y':2.5,
            'theta':0.15
        }

        # ---------------- Goal Definitions ----------------
        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [1.0, 1.0, 1.0] 
        ])
        # List of waypoints [(x, y, yaw_deg)]
        self.goals = [
            (820, 920, 0),
            (820, 1520, 0),
            (1620, 1520, 0),
            (1620, 920, 0),
            (820, 920, 0),
        ]
        #----------------DO NOT CHNAGE----------------------
        self.max_vel = 1.0
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.039, 'ki': 0.00, 'kd': 0.02, 'max_out': self.max_vel},
            'y': {'kp': 0.039, 'ki': 0.00, 'kd': 0.02, 'max_out': self.max_vel},
            'theta': {'kp': 0.01, 'ki': 0.00, 'kd': 0.01, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /bot_pose
        self.bot_sub = self.create_subscription(Poses2D,'/bot_pose', self.pose_cb,10)

        self.publisher = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz

        self.get_logger().info(f'Holonomic PID Controller started. Goals: {self.goals}')


    # ---------------- Subscriber Callback ----------------
    def pose_cb(self, msg):
        pose = msg.poses[0]
        self.current_pose = [pose.x,pose.y,pose.w]
        print(self.current_pose)

    # ---------------- Control Loop ----------------
    def control_cb(self):

        if(self.current_pose is None):
            return
        
        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        #Current pose
        bot_x, bot_y, bot_theta = self.current_pose
        bot_theta = self.convert_to_pi(bot_theta)

        # If all goals are reached → stop
        if(self.goal_index == len(self.goals)):
            self.publish_wheel_velocities([0.0,0.0,0.0])
            return
        
        # Current target goal
        goal_x, goal_y, goal_theta = self.goals[self.goal_index]
        # Errors
        error_x = goal_x - bot_x
        error_y = goal_y - bot_y
        error_theta = goal_theta - bot_theta
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
        #PID outputs
        vx = self.pid_x.compute(error_x,dt)
        vy = self.pid_y.compute(error_y,dt)
        omega = self.pid_theta.compute(error_theta,dt)
        error_matrix = np.array([vx,vy,omega])
        wheel_speed = np.linalg.solve(self.array,error_matrix)
        s1 = wheel_speed[0]
        s2 = wheel_speed[1]
        s3 = wheel_speed[2]

        # Convert to wheel velocities (custom equations)
        self.publish_wheel_velocities([s1, s2, s3])
        # Publish wheel velocities

        # Goal check
        if (abs(error_x) < self.goal_limit['x'] and 
            abs(error_y) < self.goal_limit['y'] and 
            abs(error_theta) < self.goal_limit['theta']):
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_theta.reset()
            self.goal_index += 1
            self.publish_wheel_velocities([0.0,0.0,0.0])
            return


    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        msg = Float64MultiArray()
        msg.data = np.array(wheel_vel).tolist()
        self.publisher.publish(msg)
    
    def convert_to_pi(self,angle):
        radian = math.radians(angle) 
        to_pi = math.atan2(math.sin(radian), math.cos(radian))
        return to_pi


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
