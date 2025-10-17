#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import hb_interface messages
import numpy as np
import math
from hb_interfaces.msg import Pose2D, Poses2D, BotCmd,BotCmdArray
from linkattacher_msgs.srv import AttachLink,DetachLink

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


class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        self.STATE = "ALIGN"
        # ---------------- Robot Parameters ----------------
        self.robot_id = None
        self.current_pose = None
        self.goal_index = 0
        self.last_time = self.get_clock().now()
        self.goal_limit = {
            'x': 5.0,
            'y':5.0,
            'theta':0.05
        }

        # ---------------- Goal Definitions ----------------
        self.goalpose = None
        self.d1 = [1219,1219,0]


        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [0.175, 0.175, 0.175] 
        ])

        #----------------DO NOT CHNAGE----------------------
        self.max_vel = 1.0
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.3, 'ki': 0.00, 'kd': 0.1, 'max_out': self.max_vel},
            'y': {'kp': 0.3, 'ki': 0.00, 'kd': 0.1, 'max_out': self.max_vel},
            'theta': {'kp': 0.4, 'ki': 0.00, 'kd': 0.2, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        
        # Write a subscriber for /x_pose
        self.bot_sub = self.create_subscription(Poses2D,'/bot_pose', self.bot_pose_cb,10)
        self.crate_sub = self.create_subscription(Poses2D,'/crate_pose', self.crate_pose_cb,10)

#SERVICES
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        # Wait for the service to become available
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.Attach_req = AttachLink.Request()

        self.detach_cli = self.create_client(DetachLink, '/detach_link')
        # Wait for the service to become available
        while not self.detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.Detach_req = DetachLink.Request()


        self.publisher = self.create_publisher(
            BotCmdArray, '/bot_cmd', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz



    # ---------------- Subscriber Callback ----------------
    def bot_pose_cb(self, msg):
        pose = msg.poses[0]
        self.current_pose = [pose.x,pose.y,pose.w]
        self.robot_id = pose.id
        print(self.current_pose)

    def crate_pose_cb(self,msg):
        pose = msg.poses[0]
        self.goalpose = [pose.x,pose.y,pose.w]
    
    def attach(self,botname,crate):
        self.Attach_req.model1_name = f"hb_{botname}"
        self.Attach_req.link1_name = "arm_link_2"
        self.Attach_req.model2_name = f"crate_blue_{crate}"
        self.Attach_req.link2_name = f"box_link_{crate}"
        self.future = self.attach_cli.call_async(self.Attach_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def detach(self,botname,crate):
        self.Detach_req.model1_name = f"hb_{botname}"
        self.Detach_req.link1_name = "arm_link_2"
        self.Attach_req.model2_name = f"crate_blue_{crate}"
        self.Attach_req.link2_name = f"box_link_{crate}"
        self.future = self.detach_cli.call_async(self.Detach_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



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
        goal_x, goal_y, goal_theta = self.goalpose
        dx = goal_x - bot_x
        dy = goal_y - bot_y 
        angle_to_goal = math.atan2(dy, dx)
        if self.STATE == "ALIGN":
            goal_theta = self.convert_to_pi(goal_theta)
            error_theta = -angle_to_goal - bot_theta
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            print(error_theta,",",angle_to_goal,",",bot_theta)
            self.calculate_publish(0,0,error_theta,dt)
            if (abs(error_theta + math.pi)< self.goal_limit['theta']):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.STATE = "APPROACH"

        if self.STATE == "APPROACH":
            error_x = goal_x - bot_x 
            error_y = goal_y - bot_y
            # goal_x_offset = goal_x - 130 * math.cos(goal_theta)
            # goal_y_offset = goal_y - 130 * math.sin(goal_theta)
            self.calculate_publish(error_x,error_y,0,dt)
            if (abs(error_x) < self.goal_limit['x'] and 
                abs(error_y) < self.goal_limit['y']):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0])
                return

        

        # Errors
        # dx = goal_x - bot_x
        # dy = goal_y - bot_y
        # angle_to_goal = math.atan2(dy, dx)
        # goal_x_offset = goal_x - 29.5 * math.cos(angle_to_goal)
        # goal_y_offset = goal_y - 29.5 * math.sin(angle_to_goal)

        # error_x = goal_x_offset - bot_x
        # error_y = goal_y_offset - bot_y
        # error_theta = goal_theta - bot_theta


        # Publish wheel velocities

        
    def calculate_publish(self,ex,ey,eomega,dt):

        vx = self.pid_x.compute(ex,dt)
        vy = self.pid_y.compute(ey,dt)
        omega = self.pid_theta.compute(eomega,dt)
        error_matrix = np.array([vx,vy,omega])
        wheel_speed = np.linalg.solve(self.array,error_matrix)
        s1 = wheel_speed[0]
        s2 = wheel_speed[1]
        s3 = wheel_speed[2]
        velocities = [s1,s2,s3]

        # Convert to wheel velocities (custom equations)
        self.publish_wheel_velocities(velocities)



    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self,wheel_vel):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        bot_msg_array = BotCmdArray()
        bot_msg = BotCmd()
        bot_msg.id = self.robot_id
        bot_msg.m1 = float(wheel_vel[0])
        bot_msg.m2 = float(wheel_vel[1])
        bot_msg.m3 = float(wheel_vel[2])
        bot_msg.base = float(0.0)
        bot_msg.elbow = float(90.0)
        bot_msg_array.cmds.append(bot_msg)
        self.publisher.publish(bot_msg_array)
    
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