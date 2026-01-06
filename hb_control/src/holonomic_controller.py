#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# import hb_interface messages
import numpy as np

from linkattacher_msgs.srv import AttachLink,DetachLink
import math
import json
from hb_interfaces.msg import Poses2D
from hb_interfaces.msg import BotCmdArray,BotCmd
from linkattacher_msgs.srv import AttachLink,DetachLink
from linkattacher_msgs.srv import AttachLink
# import for MQTT communication
import paho.mqtt.client as mqtt
import sys
import json
import time


class PID:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        d_error = (error - self.prev_error) / dt

        value = self.kp * error + self.kd * d_error

        self.prev_error = error
        return value

    
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0



# ---------------------- Main Node Class -------------------------------------
class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')  # initializing ros node
        # ----------MQTT-----------------
        self.broker_IP = "mqtt.local"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.MAX_WHEEL_CMD = 120.0

        try:
            self.get_logger().info(f"Connecting to MQTT Broker at {self.broker_IP}...")
            self.mqtt_client.connect(self.broker_IP, 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")                
        
    
            
        self.max_vel = 1.0
        self.robot_id = 0
        self.goal_reached = 0
        self.last_time = self.get_clock().now()
        self.current_pose = { 
                'id': 0,
                'x': 0.0,
                'y': 0.0,
                'w': 0.0
            }
        self.crate_pose = { 
                'id': 0,
                'x': 0.0,
                'y': 0.0,
                'w': 0.0
            }
        self.yaw_diff = 0.0
        self.bot_marker_dis = 0.0
        self.flag = 0
        self.state = "idle"
        self.attach_client = self.create_client(AttachLink, '/attach_link')
        self.detach_client = self.create_client(DetachLink, '/detach_link')
        self.d1 = [1330,1330,0]
        self.dock_crystal = [1218.0,205.0,0.0]
        self.link_attached = False
        self.link_detach = False
        
        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [0.175, 0.175, 0.175] 
        ])

        self.pid_params = {
            'x': {'kp': 0.29, 'ki': 0.00, 'kd': 0.06, 'max_out': self.max_vel},
            'y': {'kp': 0.29, 'ki': 0.00, 'kd': 0.06, 'max_out': self.max_vel},
            'theta': {'kp': 1.3, 'ki': 0, 'kd': 0.5, 'max_out': self.max_vel * 2}
        }

        # Initialize PIDs
        self.pid_x = PID(**self.pid_params['x'])  #this is making the new PID controller for x axis simillar for all 
        self.pid_y = PID(**self.pid_params['y'])
        self.pid_theta = PID(**self.pid_params['theta'])

        
        self.pose_subscriber = self.create_subscription(Poses2D,"/bot_pose",self.pose_cb,10)
        self.crate_subscriber = self.create_subscription(Poses2D,"/crate_pose",self.cc_pose,10)
        
        self.publisher = self.create_publisher(
            BotCmdArray, '/bot_cmd', 10
        )
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz  

    
    # MQTT main functions
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker")
            # You can subscribe to topics here
            self.mqtt_connected = True
            client.subscribe("esp/sensor/1")
        else:
            self.get_logger().error(f"MQTT Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"MQTT Message: [{msg.topic}] {msg.payload.decode()}")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("Disconnected from MQTT broker")
    
    
    def pose_cb(self, msg:Poses2D):
        if not msg.poses:  
            self.get_logger().warn("Received empty pose list from /bot_pose")
            return
        pose = msg.poses[0]  
        self.current_pose = { 
                'id': pose.id,
                'x': pose.x,
                'y': pose.y,
                'w': pose.w
            }
        



    def cc_pose(self,msg:Poses2D):
        if not msg.poses:  
            self.get_logger().warn("Received empty pose list from /crateg_pose")
            print("[WARNING] crate_pose message is EMPTY!")
            return
        pose = msg.poses[0]  
        self.crate_pose = { 
                'id': pose.id,
                'x': pose.x,
                'y': pose.y,
                'w': pose.w
            }
    
    #service for attaching the bot
    def attach(self,botname,crate):
        self.attach_req = AttachLink.Request()
        self.attach_req.data = json.dumps({
                "model1_name": "hb_crystal",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{self.crate_pose['id']}"
            })
        self.future = self.attach_client.call_async(self.attach_req)
        self.link_attached = True
    
    def detach(self,botname,crate):
        self.detach_req = DetachLink.Request()
        self.detach_req.data = json.dumps({
                "model1_name": f"hb_{botname}",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{crate}"
            })
        self.future = self.detach_client.call_async(self.detach_req)
        self.link_detach = True
    

    def compute_PID (self,ex ,ey ,etheta,dt):
        global m1,m2,m3
        vx = self.pid_x.compute(ex,dt) #pid for x and for all others is below
        vy = self.pid_y.compute(ey,dt)
        omega = self.pid_theta.compute(etheta,dt)
        error_matrix = np.array([vx,vy,omega])
        wheel_speed = np.linalg.solve(self.array,error_matrix)
        m1 = wheel_speed[0]
        m2 = wheel_speed[1]
        m3 = wheel_speed[2]
        print(abs(wheel_speed))
        m1 = self.motor_cmd_to_pwm(m1)
        m2 = self.motor_cmd_to_pwm(m2)
        m3 = self.motor_cmd_to_pwm(m3)
        self.publish_wheel_velocities(m1,m2,m3)
    
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
        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        
        goal_x = self.crate_pose['x']
        goal_y = self.crate_pose['y']
        goal_yaw = self.crate_pose['w']
        tolerance = 10.8  # units
       

        if (self.state == "idle"):
            if self.current_pose is None or self.crate_pose is None:
                return
            else:
                self.state = "Approach"
                return
            print("current state is idle")

        
        if (self.state == "Approach"):
            print("current state is Approach")
            print("this is the crate color: ",self.crate_colour(self.crate_pose['id']))
            error_x = goal_x - self.current_pose['x']
            error_y = goal_y - self.current_pose['y']
            print(goal_x, self.current_pose['x'])
            #converting world frame to bot frame 
            e_x_robot =  math.cos(self.current_pose['w'])*error_x + math.sin(self.current_pose['w'])*error_y
            e_y_robot = -math.sin(self.current_pose['w'])*error_x + math.cos(self.current_pose['w'])*error_y
            print(e_x_robot,e_y_robot)
            self.compute_PID (error_x ,error_y,0,dt)
            bot_marker_dis = math.sqrt(error_x*error_x + error_y*error_y)
            print(bot_marker_dis)
            if (bot_marker_dis <= 150):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities(0.0,0.0,0.0)
            
                self.state = "Align"
                return
        
        if (self.state == "Align"):
            self.get_logger().info("[STATE] Align")
            print("this is the crate color: ",self.crate_colour(self.crate_pose['id']))
            error_x = goal_x - self.current_pose['x']
            error_y = goal_y - self.current_pose['y']
            
            angle_to_goal = math.atan2(error_y, error_x) - math.pi/2
            error_theta = -(angle_to_goal - self.current_pose['w'])
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            
            self.compute_PID(0,0,error_theta,dt)
            bot_marker_dis = math.sqrt(error_x*error_x + error_y*error_y)
            print("error theta",abs(error_theta))
            if (bot_marker_dis > 150):
                self.state = "Approach"
                return
            if (abs(error_theta) < 0.09):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities(0.0,0.0,0.0)
                self.state = "LINK"
                print("we are in link zone")
                return    
                
    
        
        if (self.state == "LINK"):
                print("we are the linking section")
                
                print(self.attach("crystal",self.crate_pose['id']))
                self.publish_wheel_velocities(0,0.0,0.0,0.0,45.0)
                self.state = "DROPZONE"
        
        if self.state == "DROPZONE":
            goal_x, goal_y, goal_theta = self.d1
            error_x = goal_x - self.current_pose['x'] 
            error_y = goal_y - self.current_pose['y']
            angle_to_goal = math.atan2(error_x, error_y) - math.pi/2
            error_theta = -(angle_to_goal - self.current_pose['w'])
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            distance = math.sqrt(error_x*error_x + error_y*error_y)
            e_x_robot =  math.cos(self.current_pose['w'])*error_x + math.sin(self.current_pose['w'])*error_y
            e_y_robot = -math.sin(self.current_pose['w'])*error_x + math.cos(self.current_pose['w'])*error_y
            # error_theta = goal_theta-bot_theta
            self.compute_PID(e_x_robot,e_y_robot,error_theta,dt)
            print(error_x,error_y, distance)

            if (distance < 135):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.state = "DETACH"
                self.publish_wheel_velocities(0.0,0.0,0.0,45.0)
                return
        
        if (self.state == "DETACH"):
                print(self.detach("crystal",self.crate_pose['id']))
                self.publish_wheel_velocities(0.0,0.0,0.0,45.0)
                self.state = "DOCK"
            
        
        if self.state == "DOCK":
            goal_x,goal_y,goal_theta = self.dock_crystal
            error_x = goal_x - self.current_pose['x'] 
            error_y = goal_y - self.current_pose['y']
            distance = math.sqrt(error_x*error_x + error_y*error_y)
            e_x_robot =  math.cos(self.current_pose['w'])*error_x + math.sin(self.current_pose['w'])*error_y
            e_y_robot = -math.sin(self.current_pose['w'])*error_x + math.cos(self.current_pose['w'])*error_y
            # error_theta = goal_theta-bot_theta
            self.compute_PID(e_x_robot,e_y_robot,0,dt)
            distance = math.sqrt(error_x*error_x + error_y*error_y)
            print(error_x,error_y, distance)
            if (distance < 25):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities(0.0,0.0,0.0,0.0,0.0)
                self.state = "DONE"
                return
        
        if self.state == "DONE":
            self.publish_wheel_velocities(0.0,0.0,0.0,0.0,0.0)
            return
        
    def motor_cmd_to_pwm(self,cmd):
        cmd = max(-1.0, min(1.0, cmd/self.MAX_WHEEL_CMD))  

        if cmd > 0:
            # CCW
            return 1545 + cmd * (2300 - 1545)
        elif cmd < 0:
            # CW
            return 1445 + cmd * (1445 - 750)
        else:
            return 1495
        
    def publish_wheel_velocities(self,m1,m2,m3,base = 90.0,elbow = 90.0):
        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]
        bot1 = BotCmd()
        bot1.id = 0
        bot1.m1 = float(m1)
        bot1.m2 = float(m2)
        bot1.m3 = float(m3)
        bot1.base = base
        bot1.elbow = elbow 

        msg = BotCmdArray()
        msg.cmds.append(bot1)
        self.publisher.publish(msg)
        if self.mqtt_connected:
            payload = {
                    "m1" : m1,
                    "m2" : m2,
                    "m3" : m3,
                    "arm" : {"base" : -90 , "elbow" : 90},
                    "mos" : False
                    }
            self.mqtt_client.publish("esp/cmd/1", json.dumps(payload), qos=1)
            print("Sent bot wheels command")
            print("value of m1 is:", m1)
            print("value of m2 is:",m2)
            print("value of m3 is:",m3)
            self.mqtt_client.publish("esp/led","LED_ON")



# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
