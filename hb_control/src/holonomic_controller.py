#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
# import hb_interface messages
import json
import numpy as np
import math
from hb_interfaces.msg import Pose2D, Poses2D, BotCmd,BotCmdArray
from linkattacher_msgs.srv import AttachLink,DetachLink,Attach
import paho.mqtt.client as mqtt
import json

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
        # ----------MQTT-----------------
        self.broker_IP = "10.125.32.63"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_connected = False
        self.MAX = 600.0

        try:
            self.get_logger().info(f"Connecting to MQTT Broker at {self.broker_IP}...")
            self.mqtt_client.connect(self.broker_IP, 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")     
        
        
        self.STATE = "ALIGN"
        # ---------------- Robot Parameters ----------------
        self.robot_id = None
        self.current_pose = None
        self.goal_index = 0
        self.last_time = self.get_clock().now()
        self.goal_limit = {
            'x': 5.0,
            'y':5.0,
            'theta':0.1
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
            [1.0, 1.0, 1.0] 
        ])

        #----------------DO NOT CHNAGE----------------------
        self.max_vel = 1.0
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 4.7, 'ki': 0.00, 'kd': 1.2, 'max_out': self.max_vel},
            'y': {'kp': 4.7, 'ki': 0.00, 'kd': 1.2, 'max_out': self.max_vel},
            'theta': {'kp': 10.0, 'ki': 0.00, 'kd': 2.5, 'max_out': self.max_vel * 2}
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
        self.attach = self.create_service(Attach,"/Attach",self.attach_callback)
        self.ir_triggered = False

        self.publisher = self.create_publisher(
            BotCmdArray, '/bot_cmd', 10
        )
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)  # ~30ms = 33 Hz
    
    def attach_callback(self,request,response):
        if request.flag:
            status = 1
        else:
            status = 0
        robot = request.robot_id
        if not self.mqtt_connected:
            response.success = False
            return response
        
        if robot == 0:
            id = 1
        elif robot == 2:
            id = 2
        else:
            id = 3 

        print("ATTACHING LINK VIA MQTT")
        payload = {
            "status": status
        }
        self.publish_mqtt(f"esp/mos/4", payload)
        response.success = True
        return response

    # MQTT main functions
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker")
            # You can subscribe to topics here
            self.mqtt_connected = True
            client.subscribe("esp/sensor/4")
        else:
            self.get_logger().error(f"MQTT Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        if msg.payload.decode() == "0.00":
            self.ir_triggered = True
            print("IR SENSOR TRIGGERED")
        

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("Disconnected from MQTT broker")
    

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
        if self.current_pose is None or self.goalpose is None:
            return

        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        #Current pose
        bot_x, bot_y, bot_theta = self.current_pose
        print("Current Pose:",self.current_pose)
        goal_x, goal_y, goal_theta = self.goalpose
        dx = goal_x - bot_x
        dy = goal_y - bot_y 
        angle_to_goal = math.atan2(dy, dx)

        if self.STATE == "ALIGN":
            print("ALIGNING")
            # goal_x, goal_y, _ = self.d1
            bot_x, bot_y, bot_theta = self.current_pose
            # print("Current Pose:",self.current_pose)
            dx = goal_x - bot_x
            dy = goal_y - bot_y 
            angle_to_goal = math.atan2(dy, dx) - math.pi/2
            error_theta = -(angle_to_goal - bot_theta)
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
            print(error_theta)
            self.calculate_publish(0,0,error_theta,dt)
            if abs(error_theta) - bot_theta < 0.1:
            # print(angle_to_goal,error_theta,",",bot_theta)
            
            # print("IR TRIGGERED:",self.ir_triggered)
            # if error_theta < 1.5:
            #     self.pid_x.reset()
            #     self.pid_y.reset()
            #     self.pid_theta.reset()
            #     self.publish_wheel_velocities([0.0,0.0,0.0],155.0,80.0)
            #     # self.STATE = "LINK"
            #     return
            
            # self.publish_wheel_velocities([0.4,0.4,0.4],155.0,80.0)
            
        if self.STATE == "APPROACH":
            error_x = (goal_x - bot_x )
            error_y = (goal_y - bot_y)
            distance = math.sqrt(error_x*error_x + error_y*error_y)
            if (distance < 175):
                self.publish_wheel_velocities([0.0,0.0,0.0],170.0,80.0)
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.STATE = "ALIGN"
                return

            print("approach distance:",error_x,error_y, distance) 
            self.calculate_publish(error_x,error_y,0,dt)
            
        if self.STATE == "LINK":
            self.publish_wheel_velocities([0.0,0.0,0.0],180.0,80.0)
            # time.sleep(2.0) # wait for stability
            self.publish_wheel_velocities([0.0,0.0,0.0],180.0,55.0)
            time.sleep(1.0)
            req = Attach.Request()
            req.robot_id = self.robot_id
            req.flag = True
            res = Attach.Response()
            self.attach_callback(req, res)
            print(res.success)
            time.sleep(2.0)
            self.publish_wheel_velocities([0.0,0.0,0.0],155.0,55.0)
            self.STATE = "DROPZONE"
        
        if self.STATE == "DROPZONE":
            goal_x, goal_y, goal_theta = self.d1
            error_x = goal_x - bot_x 
            error_y = goal_y - bot_y

            angle_to_goal = math.atan2(error_x, error_y) - math.pi/2
            error_theta = -(angle_to_goal - bot_theta)
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

            distance = math.sqrt(error_x*error_x + error_y*error_y)
            if (distance < 50):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0],155.0,80.0)
                self.STATE = "DETACH"
                return
            
            self.calculate_publish(error_x,error_y,0,dt)
            print("Dropzone Distance", distance)
            
            
        if self.STATE == "DETACH":
            req = Attach.Request()
            req.robot_id = self.robot_id
            req.flag = False
            res = Attach.Response()
            self.attach_callback(req, res)
            print(res.success)
            self.publish_wheel_velocities([0.0,0.0,0.0],155.0,80.0)
            self.STATE = "DOCK"

        if self.STATE == "DOCK":
            goal_x,goal_y,goal_theta = self.dock_crystal
            error_x = goal_x - bot_x 
            error_y = goal_y - bot_y
            distance = math.sqrt(error_x*error_x + error_y*error_y)
            if (distance < 25):
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_theta.reset()
                self.publish_wheel_velocities([0.0,0.0,0.0],155.0,50.0)
                self.STATE = "DONE"
                return
            
            self.calculate_publish(error_x,error_y,0,dt)
            print("Dock distance", distance)
            
        
        if self.STATE == "DONE":
            print("TASK COMPLETED")
            self.publish_wheel_velocities([0.0,0.0,0.0],155.0,50.0)
            return



    def calculate_publish(self, ex, ey, eomega, dt):
        if dt <= 1e-4:
            return
    
        _,_,bot_theta = self.current_pose
        vx = self.pid_x.compute(ex, dt)
        vy = self.pid_y.compute(ey, dt)
        omega = self.pid_theta.compute(eomega, dt)

        x_std =  math.cos(bot_theta)*vx + math.sin(bot_theta)*vy
        y_std = -math.sin(bot_theta)*vx + math.cos(bot_theta)*vy
        
        wheel_vel = np.linalg.pinv(self.array) @ np.array([-x_std, y_std, omega])

        wheel_vel = wheel_vel/self.MAX
        self.publish_wheel_velocities(wheel_vel, 170.0)
    
    
    def publish_mqtt(self, topic,payload):
        self.mqtt_client.publish(topic, json.dumps(payload), qos=1,retain=False)



    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self, wheel_vel=[0.0,0.0,0.0],arm_state=155.0, elbow=80.0):
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
        if self.mqtt_connected:
            payload = {
                "m1": wheel_vel[0],
                "m2": wheel_vel[1],
                "m3": wheel_vel[2],
                "arm": {"base": arm_state, "elbow": 80.0}
            }
            self.publish_mqtt("esp/cmd/4", payload)
            print("MQTT CMD:", wheel_vel)

        # Wheel velocity array (Float64MultiArray)
        # Order: [Left wheel speed, Right wheel speed, Rear wheel speed]


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    controller.publish_wheel_velocities()
    rclpy.shutdown()


if __name__ == '__main__':
    main()