#!/usr/bin/env python3
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
        self.integral += error * dt
        kd_error = (error - self.prev_error)/dt
        value = error*self.kp + kd_error*self.kd + self.ki * self.integral
        self.prev_error = error
        return value
    
    def reset(self):
        print("[PID] Resetting PID controller")
        self.integral = 0.0
        self.prev_error = 0.0


class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')
        # ---------------- MQTT Client Setup ----------------
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

        
        
        self.goal_limit = {
            'x': 5.0,
            'y':5.0,
            'theta':0.09
        }

        # ---------------- Goal Definitions ----------------
        self.d1 = [1219,1200,0]
        self.d2 = [820,2000.5,0]
        self.d3= [1616,2000.5,0]
        self.dock_crystal = [1218.0,205.0,0.0]
        
        self.crate_positions = {}
        self.assigned_crates = set()
        self.unassigned_crates = []
        self.crate_assignment_done = False

        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [1.0, 1.0, 1.0] 
        ])

        self.max_vel = 1.0
        
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 4.7, 'ki': 0.0, 'kd': 1.2, 'max_out': self.max_vel},
            'y': {'kp': 4.7, 'ki': 0.0, 'kd': 1.2, 'max_out': self.max_vel},
            'theta': {'kp': 3.0, 'ki': 0.1, 'kd': 0.3, 'max_out': self.max_vel * 2}
        }
        
        self.bots = {
            0: {
                "state": "APPROACH",
                "name": "crystal",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[1218.0,205.0,0.0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False,
                
            },
            2: { 
                "state": "APPROACH",
                "name": "frostbite",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[1568.0,202.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False
            },
            4: { 
                "state": "APPROACH",
                "name": "glacio",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[864.0,204.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False
            }
        }

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        self.bot_sub = self.create_subscription(Poses2D,'/bot_pose', self.bot_pose_cb,10)
        self.crate_sub = self.create_subscription(Poses2D,'/crate_pose', self.crate_pose_cb,10)
        #SERVICES
        self.attach = self.create_service(Attach,"/Attach",self.attach_callback)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)

    def attach_callback(self,request,response):
        if request.flag:
            status = 1
        else:
            status = 0
        robot = request.robot_id
        if not self.mqtt_connected:
            response.success = False
            return response

        print("ATTACHING LINK VIA MQTT")
        payload = {
            "status": status
        }
        self.publish_mqtt(f"esp/mos/{robot}", payload)
        response.success = True
        return response
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT Broker")
            # You can subscribe to topics here
            self.mqtt_connected = True
            client.subscribe("esp/sensor/0")
            client.subscribe("esp/sensor/2")
            client.subscribe("esp/sensor/4")
        else:
            self.get_logger().error(f"MQTT Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()
        bot_id = int(topic.split("/")[-1])
        self.bots[bot_id]["ir_triggered"] = (payload == "0.00")

    def bot_pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id in self.bots:
                self.bots[pose.id]["pose"] = [pose.x,pose.y,pose.w]
    
    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("Disconnected from MQTT broker")

    def crate_pose_cb(self, msg):
        for pose in msg.poses:
            crate_id = pose.id
            self.crate_positions[crate_id] = [pose.x, pose.y, pose.w]

        # Update goal if already assigned
        for bot_id, bot_data in self.bots.items():
            if bot_data["crate_id"] == crate_id:
                bot_data["goal"] = [pose.x, pose.y, pose.w]
                    

        # Try assigning crates if not done
        if not self.crate_assignment_done and len(self.crate_positions) > 0:
            self.assign_crates_to_bots()
            self.crate_assignment_done = True

    
    def crate_colour(self,id):
        a = id%3
        if a == 0:
            color = "red"
        elif a == 1:
            color = "green"
        else:
            color = "blue"
        return color
    
    def assign_crates_to_bots(self):
        time.sleep(0.5)  # Ensure all crate positions are updated
        print("\n=== Assigning crates to bots ===")

        assigned = set()

        bot_positions = {
            bot_id: bot_data["pose"]
            for bot_id, bot_data in self.bots.items()
            if bot_data["pose"] is not None
        }

        print(f"[DEBUG] Active bots with pose: {bot_positions}")

        if not bot_positions:
            print("[DEBUG] No bot poses available. Exiting assignment.")
            return

        available_crates = {
            cid: cpose for cid, cpose in self.crate_positions.items() if cid not in self.assigned_crates
        }

        print(f"[DEBUG] Available crates: {available_crates}")

        for bot_id, bot_pose in bot_positions.items():
            if not available_crates:
                print(f"[DEBUG] No more crates available to assign.")
                break

            bx, by, _ = bot_pose
            print(f"[DEBUG] Checking bot {bot_id} at ({bx:.2f}, {by:.2f})")

            nearest_crate, nearest_dist = None, float('inf')

            for cid, (cx, cy, _) in available_crates.items():
                dist = math.sqrt((cx - bx)**2 + (cy - by)**2)
                print(f"   -> crate {cid} at ({cx:.2f}, {cy:.2f}) dist={dist:.3f}")

                if dist < nearest_dist:
                    nearest_crate, nearest_dist = cid, dist

            if nearest_crate is not None:
                print(f"[DEBUG] Bot {bot_id} assigned crate {nearest_crate} (dist={nearest_dist:.3f})")
                self.bots[bot_id]["crate_id"] = nearest_crate
                self.bots[bot_id]["goal"] = available_crates[nearest_crate]
                assigned.add(nearest_crate)
                del available_crates[nearest_crate]

        remaining = set(self.crate_positions.keys()) - self.assigned_crates - assigned
        self.unassigned_crates = list(remaining)

        self.assigned_crates |= assigned

        print(f"[DEBUG] Assigned crates so far: {self.assigned_crates}")
        print(f"[DEBUG] Remaining unassigned crates: {self.unassigned_crates}")
        print("=== Assignment Complete ===\n")


    # ---------------- Control Loop ----------------
    def control_cb(self):
        for bot_id,bot_data in self.bots.items():
            name = bot_data["name"]
            state = bot_data["state"]
            botPose = bot_data["pose"]
            goalPose = bot_data["goal"]
            crate = bot_data["crate_id"]
            last_time = bot_data["last_time"]
            dock = bot_data["dock"]
            self.pid_x = bot_data["pid_x"]
            self.pid_y = bot_data["pid_y"]
            self.pid_theta = bot_data["pid_theta"]

            if(botPose is None):
                continue  
            
            if(goalPose is None):
                continue  
            
            # Time delta
            now = self.get_clock().now()
            dt = (now - last_time).nanoseconds / 1e9
            if dt <= 0:
                continue 
            bot_data["last_time"] = now  
            
            #Current pose
            bot_x, bot_y, bot_theta = botPose
            goal_x, goal_y, _ = goalPose
            dx = goal_x - bot_x
            dy = goal_y - bot_y 
            angle_to_goal = math.atan2(dy, dx) - math.pi/2

            if state == "ALIGN":
                error_theta = -(angle_to_goal - bot_theta)
                print("IR TRIGGERED:",bot_data["ir_triggered"])
                if bot_data["ir_triggered"]:
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],155.0,80.0)
                    bot_data["ir_triggered"] = False
                    bot_data["state"] = "LINK"
                    return
            # self.calculate_publish(0,0,30.0,dt)
                self.publish_wheel_velocities(bot_id,[0.4,0.4,0.4],180.0,80.0)
                
            if state == "APPROACH":
                error_x = (goal_x - bot_x )
                error_y = (goal_y - bot_y)
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                if (distance < 180):
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],180.0,80.0)
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    bot_data["state"] = "ALIGN"
                    return

                print("approach distance:",error_x,error_y, distance) 
                self.calculate_publish(error_x,error_y,0,dt,bot_id)
                
            if state == "LINK":
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],180.0,80.0)
                # time.sleep(2.0) # wait for stability
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],180.0,55.0)
                time.sleep(1.0)
                req = Attach.Request()
                req.robot_id = bot_id
                req.flag = True
                res = Attach.Response()
                self.attach_callback(req, res)
                print(res.success)
                time.sleep(2.0)
                bot_data["state"] = "DROPZONE"
            
            if state== "DROPZONE":
                dropzones = {"red": self.d1, "green": self.d2, "blue": self.d3}
                goal_x, goal_y, _ = dropzones[self.crate_colour(crate)]
                error_x = goal_x - bot_x 
                error_y = goal_y - bot_y

                distance = math.sqrt(error_x*error_x + error_y*error_y)
                if (distance < 20):
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],155.0,80.0)
                    bot_data["state"] = "DETACH"
                    return
                
                self.calculate_publish(error_x,error_y,0,dt,bot_id)
                print("Dropzone Distance", distance)
                
            if state == "DETACH":
                req = Attach.Request()
                req.robot_id = bot_id
                req.flag = False
                res = Attach.Response()
                self.attach_callback(req, res)
                print(res.success)
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],155.0,80.0)
                if self.unassigned_crates:
                        left_crate = self.unassigned_crates.pop(0)
                        bot_data["crate_id"] = left_crate
                        bot_data["goal"] = self.crate_positions[left_crate]
                        bot_data["link_attached"] = False 
                        bot_data["link_detach"] = False
                        bot_data["state"] = "APPROACH"
                else: 
                    bot_data["state"] = "DOCK"
                continue  


            if state == "DOCK":
                goal_x,goal_y,_ = dock
                error_x = goal_x - bot_x 
                error_y = goal_y - bot_y
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                if (distance < 25):
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],155.0,50.0)
                    bot_data["state"] = "DONE"
                    return
                
                self.calculate_publish(error_x,error_y,0,dt,bot_id)
                print("Dock distance", distance)
            
            if state == "DONE":
                print(f"Bot {name} has completed its task.")
                continue 

    def calculate_publish(self, ex, ey, eomega, dt,id):
        if dt <= 1e-4:
            return
    
        _,_,bot_theta = self.bots[id]["pose"]
        vx = self.pid_x.compute(ex, dt)
        vy = self.pid_y.compute(ey, dt)
        omega = self.pid_theta.compute(eomega, dt)

        x_std =  math.cos(bot_theta)*vx + math.sin(bot_theta)*vy
        y_std = -math.sin(bot_theta)*vx + math.cos(bot_theta)*vy
        
        wheel_vel = np.linalg.pinv(self.array) @ np.array([-x_std, y_std, omega])

        wheel_vel = wheel_vel/self.MAX
        self.publish_wheel_velocities(id,wheel_vel, 180.0, 80.0)
    
    
    def publish_mqtt(self, topic,payload):
        self.mqtt_client.publish(topic, json.dumps(payload), qos=1,retain=False)

    def publish_wheel_velocities(self, id,wheel_vel=[0.0,0.0,0.0],arm_state=155.0, elbow=80.0):
        bot_msg_array = BotCmdArray()
        bot_msg = BotCmd()
        bot_msg.id = id
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
            self.publish_mqtt(f"esp/cmd/{id}", payload)
            print("MQTT CMD:", wheel_vel)




# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()