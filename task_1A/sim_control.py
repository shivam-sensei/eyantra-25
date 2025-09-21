#!/usr/bin/env python3
import rclpy
import ast
from rclpy.node import Node
import math
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from eyantrasim_msgs.msg import Pose

class BattalionController(Node):
    def __init__(self):
        super().__init__('battalion_controller')

        # --- Service client (for targets) ---
        self.cli = self.create_client(Trigger, '/eyantrasim/get_coordinates')
        self.req = Trigger.Request()

        # --- Publishers (for bots) ---
        self.pub_glacio = self.create_publisher(Twist, '/eyantrasim/glacio/cmd_vel', 10)
        self.pub_crystal = self.create_publisher(Twist, '/eyantrasim/crystal/cmd_vel', 10)
        self.pub_frostbite = self.create_publisher(Twist, '/eyantrasim/frostbite/cmd_vel', 10)

        # --- Subscribers (for bot poses) ---
        self.sub_glacio = self.create_subscription(Pose, '/eyantrasim/glacio/pose', self.pose_glacio_cb, 10)
        self.sub_crystal = self.create_subscription(Pose, '/eyantrasim/crystal/pose', self.pose_crystal_cb, 10)
        self.sub_frostbite = self.create_subscription(Pose, '/eyantrasim/frostbite/pose', self.pose_frostbite_cb, 10)

        # --- Storage placeholders ---
        self.targets_glacio = []
        self.targets_crystal = []
        self.targets_frostbite = []
        self.previous_theta = {1:0.0 , 2:0.0, 3:0.0}
        self.current_pose_glacio = None
        self.current_pose_crystal = None
        self.current_pose_frostbite = None
        self.num = 0
        self.num1 = 0
        self.num2 = 0


        cords = self.get_targets()
        self.targets_glacio = cords[0:50]
        self.targets_crystal = cords[50:100]
        self.targets_frostbite = cords[100:150]
        # --- Timer for control loop ---
        self.timer = self.create_timer(0.1, self.control_loop)

    # --- Example method: request targets from service ---
    def get_targets(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,future)
        return(ast.literal_eval(future.result().message))

    # --- Pose callbacks ---
    def pose_glacio_cb(self, msg):
        self.current_pose_glacio = msg

    def pose_crystal_cb(self, msg):
        self.current_pose_crystal = msg

    def pose_frostbite_cb(self, msg):
        self.current_pose_frostbite = msg

    # # --- Example controller ---
    def compute_velocity(self, current_pose, target_pose, bot):
        vel = Twist()
        dx = target_pose[0] - current_pose.x
        dy = target_pose[1] - current_pose.y
        distance = math.sqrt((dx*dx + dy*dy))
        desired_theta =self.angle_restrict(math.atan2(dy,dx) - current_pose.theta)
        error_theta = (desired_theta - self.previous_theta[bot]) / 0.1
        self.previous_theta[bot] = desired_theta
        kp_distance = 0.222
        kp_theta = 1.11
        kd_theta = 0.34
        angle = kp_theta * desired_theta + (kd_theta * error_theta)
        if distance < 0.01:
            print (target_pose)
            if bot == 1:
                self.num = min(49,self.num+1)
            elif bot == 2:
                self.num1 = min(49,self.num1+1)
            else:
                self.num2 = min(49,self.num2+1)
        if abs(desired_theta) > 0.09:
            v = 0.0
        else:
            v = kp_distance * distance
            if v > 0.55:
                v = 1.11
        
        vel.linear.x = v
        vel.angular.z = angle
        return vel
    
    def angle_restrict(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # # --- Control loop for all bots ---
    def control_loop(self):
        if self.current_pose_glacio and self.targets_glacio:
            vel = self.compute_velocity(self.current_pose_glacio, self.targets_glacio[self.num],1)
            self.pub_glacio.publish(vel)
            vel = self.compute_velocity(self.current_pose_crystal, self.targets_crystal[self.num1],2)
            self.pub_crystal.publish(vel)
            vel = self.compute_velocity(self.current_pose_frostbite, self.targets_frostbite[self.num2],3)
            self.pub_frostbite.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = BattalionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()