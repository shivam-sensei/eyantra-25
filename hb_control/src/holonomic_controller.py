#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import hb_interface messages
import json
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
        if value < 20.0 and value > 5:
            value *= abs(25-value)*0.4
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
            'theta':0.09
        }

        # ---------------- Goal Definitions ----------------
        self.goalpose = None
        self.d1 = [1219,1219,0]
        self.dock_crystal = [1218.0,205.0,0.0]
        self.crate_id = None
        self.link_attached = False
        self.link_detach = False


        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [0.175, 0.175, 0.175] 
        ])

        #----------------DO NOT CHNAGE----------------------
        self.max_vel = 1.0
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.3, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'y': {'kp': 0.3, 'ki': 0.00, 'kd': 0.05, 'max_out': self.max_vel},
            'theta': {'kp': 1.1, 'ki': 0.00, 'kd': 0.6, 'max_out': self.max_vel * 2}
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
        # print(self.current_pose)

    def crate_pose_cb(self,msg):
        pose = msg.poses[0]
        self.goalpose = [pose.x,pose.y,pose.w]
        self.crate_id = pose.id
    
    def attach(self,botname,crate):
        self.Attach_req.data = json.dumps({
                "model1_name": f"hb_{botname}",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{crate}"
            })
        self.future = self.attach_cli.call_async(self.Attach_req)
        self.link_attached = True
        
        
        

    def detach(self,botname,crate):
        self.Detach_req.data = json.dumps({
                "model1_name": f"hb_{botname}",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{crate}"
            })
        self.future = self.detach_cli.call_async(self.Detach_req)
        self.link_detach = True
    
    def crate_colour(self,id):
        a = id%3
        if a == 0:
            return "red"
        elif a == 1:
            return "green"
        elif a == 2:
            return "blue"



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
        angle_to_goal = math.atan2(dy, dx) - math.pi/2
        if self.STATE == "ALIGN":
            goal_theta = self.convert_to_pi(goal_theta)
            error_theta = -(angle_to_goal - bot_theta)
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            print(error_theta,",",angle_to_goal,",",bot_theta)
            self.calculate_publish(0,0,error_theta,dt)
            if (abs(error_theta)< self.goal_limit['theta']):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0])
                self.STATE = "APPROACH"
                return
        angle_to_goal += math.pi/2
        if self.STATE == "APPROACH":
            
            goal_x_offset = goal_x - 35 * math.cos(angle_to_goal)
            goal_y_offset = goal_y - 35 * math.sin(angle_to_goal)
            error_x = goal_x_offset - bot_x 
            error_y = goal_y_offset - bot_y
            e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
            e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
            print(e_x_robot,e_y_robot, error_x,error_y) 
            self.calculate_publish(e_x_robot,e_y_robot,0,dt)
            if (abs(e_x_robot) < 15.0 and 
                abs(e_y_robot) < 130.0):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0],90.0)
                self.STATE = "LINK"
                return
            
        if self.STATE == "LINK":
            if not self.link_attached:
                print(self.attach("crystal",self.crate_id))
            self.publish_wheel_velocities([0.0,0.0,0.0],45.0)
            self.STATE = "DROPZONE"
        
        if self.STATE == "DROPZONE":
            goal_x, goal_y, goal_theta = self.d1
            error_x = goal_x - bot_x 
            error_y = goal_y - bot_y
            e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
            e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
            # error_theta = goal_theta-bot_theta
            self.calculate_publish(e_x_robot,e_y_robot,0,dt)
            print(e_x_robot,e_y_robot) 
            if (abs(error_x) < 5.0 and 
                abs(error_y) < 5.0):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0],45.0)
                self.STATE = "DETACH"
                return
            
        if self.STATE == "DETACH":
            if not self.link_detach:
                print(self.detach("crystal",self.crate_id))
            self.publish_wheel_velocities([0.0,0.0,0.0],45.0)
            self.STATE = "DOCK"

        if self.STATE == "DOCK":
            goal_x,goal_y,goal_theta = self.dock_crystal
            error_x = goal_x - bot_x 
            error_y = goal_y - bot_y
            e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
            e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
            # error_theta = goal_theta-bot_theta
            self.calculate_publish(e_x_robot,e_y_robot,0,dt)
            print(error_x,error_y) 
            if (abs(error_x) < 5.0 and 
                abs(error_y) < 5.0):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0],0.0,0.0)
                self.STATE = "DONE"
                return
        
        if self.STATE == "DONE":
            return



        
    def calculate_publish(self,ex,ey,eomega,dt):
        if abs(ex) < self.goal_limit['x']: ex = 0.0
        if abs(ey) < self.goal_limit['y']: ey = 0.0
        if abs(eomega) < self.goal_limit['theta']: eomega = 0.0
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
        self.publish_wheel_velocities(velocities,90.0)



    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self,wheel_vel=[0.0,0.0,0.0],arm_state = 0.0, elbow = 90.0):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        bot_msg_array = BotCmdArray()
        bot_msg = BotCmd()
        bot_msg.id = self.robot_id
        bot_msg.m1 = float(wheel_vel[0])
        bot_msg.m2 = float(wheel_vel[1])
        bot_msg.m3 = float(wheel_vel[2])
        bot_msg.base = float(arm_state)
        bot_msg.elbow = float(elbow)
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