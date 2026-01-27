#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import numpy as np
import math
from hb_interfaces.msg import Poses2D, BotCmd,BotCmdArray
from linkattacher_msgs.srv import AttachLink,DetachLink,Attach
import paho.mqtt.client as mqtt
import json
from scipy.optimize import linear_sum_assignment
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
        self.broker_IP = "localhost"
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_connected = False
        self.MAX = 650.0
        self.last_print_time = self.get_clock().now()
        self.crateAlignCounter =0

        self.zone_lock = {
            "red": None,
            "green": None,
            "blue": None
        }
        self.dropzone_slots = {"red": [], "green": [], "blue": []}
        self.dropzone_offsets = {
            "red":  [(-200,60), (170,-90), (0,0)],
            "blue": [(160,0), (-160,0), (0,0)],
            "green":[(160,0), (-160,0), (0,0)]
        }
        self.dropzone_count = {
            "red": 0,
            "blue": 0,
            "green": 0
        }

        self.dropped_crates = []

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
        self.d1 = [1210,1220,0]
        self.d2 = [820,2036,0]
        self.d3= [1618,2036,0]
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
            'x': {'kp': 4.7, 'ki': 0.0, 'kd': 1.7, 'max_out': self.max_vel},
            'y': {'kp': 4.7, 'ki': 0.0, 'kd': 1.7, 'max_out': self.max_vel},
            'theta': {'kp': 100.0, 'ki': 0.1, 'kd': 5.5, 'max_out': self.max_vel * 2}
        }
        
        self.bots = {
            0: {
                "state": "APPROACH",
                "name": "crystal",      #purple
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[1218.0,200.0,0.0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False,
                "Default_arm_position": {"base": 160.0, "elbow": 80.0},
                "Pick_arm_position": {"base": 176.0, "elbow": 80.0},
                "distance":150,
                "drop_slot":None
                
            },
            2: { 
                "state": "APPROACH",
                "name": "frostbite",    #red
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[1568.0,202.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False,
                "Default_arm_position": {"base": 50.0, "elbow": 25.0},
                "Pick_arm_position": {"base": 75.0, "elbow": 25.0},
                "distance":175,
                "drop_slot":None
            },
            4: { 
                "state": "APPROACH",
                "name": "glacio",       #pink
                "pose": None,
                "goal": None,
                "crate_id": None,
                "dock":[864.0,204.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now(),
                "ir_triggered": False,
                "Default_arm_position": {"base": 155.0, "elbow": 80.0},
                "Pick_arm_position": {"base": 180.0, "elbow": 80.0},
                "distance":140,
                "drop_slot":None
            }
        }

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        self.bot_sub = self.create_subscription(Poses2D,'/bot_pose', self.bot_pose_cb,10)
        self.bot_pose_cb
        self.crate_sub = self.create_subscription(Poses2D,'/crate_pose', self.crate_pose_cb,10)
        #SERVICES
        self.attach = self.create_service(Attach,"/Attach",self.attach_callback)
        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.05, self.control_cb)

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
    
    def assign_dropzone_slot(self, bot_id, color):
        if bot_id not in self.dropzone_slots[color]:
            if len(self.dropzone_slots[color]) < len(self.dropzone_offsets[color]):
                self.dropzone_slots[color].append(bot_id)

    
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
        allowed = {12,30,16,13}
        for pose in msg.poses:
            crate_id = pose.id
            if crate_id not in allowed:
                continue
            self.crate_positions[crate_id] = [pose.x, pose.y, pose.w]

            # Update goal if already assigned
            for bot_id, bot_data in self.bots.items():
                if bot_data["crate_id"] == crate_id:
                    bot_data["goal"] = [pose.x, pose.y, pose.w]
                    

        # Try assigning crates if not done
        if  (not self.crate_assignment_done
            and self.crate_positions
            and any(bot["pose"] is not None for bot in self.bots.values())):
            self.assign_crates_to_bots()
            print("Assigning crates")
            self.crate_assignment_done = True

    
    def crate_colour(self,id):
        a = id%3
        if a == 0:
            color = "red"
            print("red")
        elif a == 1:
            color = "green"
            print("green")
        else:
            color = "blue"
            print("blue")
        return color
    
    def assign_crates_to_bots(self):
        # collect bot positions (non-blocking)
        bot_positions = {
            bot_id: bot_data["pose"]
            for bot_id, bot_data in self.bots.items()
            if bot_data["pose"] is not None
        }

        if not bot_positions:
            self.get_logger().warn(
                "[ASSIGN] No bot poses yet, skipping crate assignment"
            )
            return

        available_crates = {
            cid: cpose
            for cid, cpose in self.crate_positions.items()
            if cid not in self.assigned_crates
        }

        if not available_crates:
            self.get_logger().warn("[ASSIGN] No crates available")
            return

        bots = list(bot_positions.keys())
        crates = list(available_crates.keys())

        cost = np.zeros((len(bots), len(crates)), dtype=float)

        for i, bot_id in enumerate(bots):
            bx, by, _ = bot_positions[bot_id]
            for j, cid in enumerate(crates):
                cx, cy, _ = available_crates[cid]
                cost[i, j] = math.hypot(cx - bx, cy - by)

        row_ind, col_ind = linear_sum_assignment(cost)

        for r, c in zip(row_ind, col_ind):
            bot_id = bots[r]
            crate_id = crates[c]

            self.bots[bot_id]["crate_id"] = crate_id
            self.bots[bot_id]["goal"] = available_crates[crate_id]
            self.assigned_crates.add(crate_id)

            self.get_logger().info(
                f"[ASSIGN] Bot {bot_id} → Crate {crate_id} (cost={cost[r,c]:.2f})"
            )

        self.unassigned_crates = list(
            set(self.crate_positions.keys()) - self.assigned_crates
        )

        self.crate_assignment_done = True

    # ---------------- Control Loop ----------------
    def control_cb(self):
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds > 2e9: # Print every 2 seconds
            for bid, bdata in self.bots.items():
                self.get_logger().info(f"STATUS: Bot {bid} chasing Crate {bdata['crate_id']}")
                self.last_print_time = now

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
            default_arm = bot_data["Default_arm_position"]
            pick_arm = bot_data["Pick_arm_position"]

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
            # collect all bot positions
            bot_positions = {bid: bdata["pose"] for bid, bdata in self.bots.items() if bdata["pose"] is not None}


            if state == "ALIGN":
                error_theta = -(angle_to_goal - bot_theta)
                print("IR TRIGGERED:",bot_data["ir_triggered"])
                if bot_data["ir_triggered"]:
                    if self.crateAlignCounter <6:
                        self.crateAlignCounter +=1
                        continue
                    else:
                        # if bot_id == 4:
                        #     time.sleep(0.35)
                        self.pid_x.reset() 
                        self.pid_y.reset()
                        self.pid_theta.reset()
                        self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],pick_arm["base"],pick_arm["elbow"])
                        bot_data["ir_triggered"] = False
                        self.crateAlignCounter =0
                        bot_data["state"] = "LINK"
                        return
            # self.calculate_publish(0,0,30.0,dt)
                self.publish_wheel_velocities(bot_id,[0.4,0.4,0.4],default_arm["base"],default_arm["elbow"])
                
            if state == "APPROACH":
                dist = bot_data["distance"]
                error_x = (goal_x - bot_x )
                error_y = (goal_y - bot_y)
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                if (distance < dist):
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    bot_data["state"] = "ALIGN"
                    return
                
                for id in self.dropped_crates:
                    if id == crate:
                        continue

                    if id in self.crate_positions:
                        cx,cy,_ = self.crate_positions[id]
                        avoid_radius = 250.0 
                        dx_c = bot_x - cx
                        dy_c = bot_y - cy
                        dist = math.sqrt(dx_c*dx_c + dy_c*dy_c)
                        if dist < avoid_radius and dist > 1.0:
                            rep = (avoid_radius - dist) / avoid_radius
                            avoid_gain = 1900.0   # tune
                            error_x += avoid_gain * rep * (dx_c / dist)
                            error_y += avoid_gain * rep * (dy_c / dist)

                
                # ---- Bot-to-Bot repulsion (vector-based) ----
                avoid_bot_radius = 300.0     # influence radius
                avoid_bot_gain   = 1900.0    # same scale as crate repulsion, tune separately

                for other_id, other_pose in bot_positions.items():
                    if other_id == bot_id:
                        continue

                    ox, oy, _ = other_pose
                    dx_b = bot_x - ox     # note: vector away from other bot
                    dy_b = bot_y - oy
                    dist_b = math.sqrt(dx_b*dx_b + dy_b*dy_b)

                    if dist_b < avoid_bot_radius and dist_b > 1.0:
                        rep = (avoid_bot_radius - dist_b) / avoid_bot_radius
                        error_x += avoid_bot_gain * rep * (dx_b / dist_b)
                        error_y += avoid_bot_gain * rep * (dy_b / dist_b)

                print("approach distance:",distance,"crate:",crate) 
                self.calculate_publish(error_x, error_y, 0, dt, bot_id, default_arm["base"], default_arm["elbow"])

                
            if state == "LINK":
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],pick_arm["base"],pick_arm["elbow"])
                req = Attach.Request()
                req.robot_id = bot_id
                req.flag = True
                res = Attach.Response()
                self.attach_callback(req, res)
                print(res.success)
                time.sleep(1.0)
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                color = self.crate_colour(crate)

                # assign slot ONCE
                if bot_data["drop_slot"] is None:
                    self.assign_dropzone_slot(bot_id, color)
                    bot_data["drop_slot"] = self.dropzone_slots[color].index(bot_id)
                    print(f"[SLOT ASSIGNED] Bot {bot_id} → {color} slot {bot_data['drop_slot']}")

                bot_data["state"] = "DROPZONE"

            color = self.crate_colour(crate)
            dropzones = {"red": self.d1, "green": self.d2, "blue": self.d3}

            if state== "DROPZONE":
                if color == "red" and self.dropzone_count[color] == 2:
                    bot_data["state"] = "STACK"
                    continue
                goal_x, goal_y, _ = dropzones[color]
                idx = self.dropzone_count[color]
                ox, oy = self.dropzone_offsets[color][idx]

                if dropzones[color] in [self.d2, self.d3]:
                    goal_x = dropzones[color][0] + ox
                    goal_y = dropzones[color][1] + oy
                else:
                    goal_x = dropzones[color][0] + ox
                    goal_y = dropzones[color][1] + oy
                # === ZONE LOCK (gives exclusive access only for the final align/detach) ===
                if self.zone_lock[color] not in (None, bot_id):
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                    continue

                if self.zone_lock[color] is None:
                    self.zone_lock[color] = bot_id
                
                error_x = goal_x - bot_x
                error_y = goal_y - bot_y
                distance = math.hypot(error_x, error_y)

                if (distance < 20):
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                    bot_data["state"] = "ZONE_ALIGN"
                    continue

                for id in self.dropped_crates:
                    if id == crate:
                        continue

                    if id in self.crate_positions:
                        cx,cy,_ = self.crate_positions[id]
                        avoid_radius = 120.0 
                        dx_c = bot_x - cx
                        dy_c = bot_y - cy
                        dist = math.sqrt(dx_c*dx_c + dy_c*dy_c)
                        if dist < avoid_radius and dist > 1.0:
                            rep = (avoid_radius - dist) / avoid_radius
                            avoid_gain = 1700.0   # tune
                            error_x += avoid_gain * rep * (dx_c / dist)
                            error_y += avoid_gain * rep * (dy_c / dist)
                # ---- Bot-to-Bot repulsion (vector-based) ----
                avoid_bot_radius = 300.0     # influence radius
                avoid_bot_gain   = 1900.0    # same scale as crate repulsion, tune separately

                for other_id, other_pose in bot_positions.items():
                    if other_id == bot_id:
                        continue
                    ox, oy, _ = other_pose
                    dx_b = bot_x - ox     # note: vector away from other bot
                    dy_b = bot_y - oy
                    dist_b = math.sqrt(dx_b*dx_b + dy_b*dy_b)

                    if dist_b < avoid_bot_radius and dist_b > 1.0:
                        rep = (avoid_bot_radius - dist_b) / avoid_bot_radius
                        error_x += avoid_bot_gain * rep * (dx_b / dist_b)
                        error_y += avoid_bot_gain * rep * (dy_b / dist_b)

                print("approach distance:",distance,"crate:",crate) 
                self.calculate_publish(error_x, error_y, 0, dt, bot_id, default_arm["base"], default_arm["elbow"])
                print("Dropzone Distance", distance)

            if state == "STACK":
                if "stack_phase" not in bot_data:
                    bot_data["stack_phase"] = "APPROACH"
                
                if bot_data["stack_phase"] == "APPROACH":
                    mid_x, mid_y = self.computemidpoint()
                    error_x = mid_x - bot_x
                    error_y = mid_y - bot_y
                    distance = math.hypot(error_x, error_y)

                    if distance < 20:
                        bot_data["stack_phase"] = "ALIGN"
                        self.pid_x.reset()
                        self.pid_y.reset()
                        self.pid_theta.reset()
                        self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                        return
                    
                    for id in self.dropped_crates:
                        if id == crate:
                            continue

                        if id in self.crate_positions:
                            cx,cy,_ = self.crate_positions[id]
                            avoid_radius = 120.0 
                            dx_c = bot_x - cx
                            dy_c = bot_y - cy
                            dist = math.sqrt(dx_c*dx_c + dy_c*dy_c)
                            if dist < avoid_radius and dist > 1.0:
                                rep = (avoid_radius - dist) / avoid_radius
                                avoid_gain = 1700.0   # tune
                                error_x += avoid_gain * rep * (dx_c / dist)
                                error_y += avoid_gain * rep * (dy_c / dist)
                    # ---- Bot-to-Bot repulsion (vector-based) ----
                    avoid_bot_radius = 300.0     # influence radius
                    avoid_bot_gain   = 1900.0    # same scale as crate repulsion, tune separately

                    for other_id, other_pose in bot_positions.items():
                        if other_id == bot_id:
                            continue
                        ox, oy, _ = other_pose
                        dx_b = bot_x - ox     # note: vector away from other bot
                        dy_b = bot_y - oy
                        dist_b = math.sqrt(dx_b*dx_b + dy_b*dy_b)

                        if dist_b < avoid_bot_radius and dist_b > 1.0:
                            rep = (avoid_bot_radius - dist_b) / avoid_bot_radius
                            error_x += avoid_bot_gain * rep * (dx_b / dist_b)
                            error_y += avoid_bot_gain * rep * (dy_b / dist_b)
                    self.calculate_publish(error_x, error_y, 0, dt, bot_id, default_arm["base"], default_arm["elbow"])
                    print("Stacking Approach Distance:", distance)
                
                if bot_data["stack_phase"] == "ALIGN":
                    goal_x, goal_y, _ = self.computemidpoint()
                    dx = goal_x - bot_x
                    dy = goal_y - bot_y
                    angle_to_goal = math.atan2(dy, dx) + math.pi/2   # matches your approach convention
                    error_theta = math.atan2(math.sin(angle_to_goal - bot_theta), math.cos(angle_to_goal - bot_theta))

                    omega = Kp_theta * abs(error_theta)
                    omega = min(max_w, max(min_w, omega))
                    omega *= math.copysign(1, error_theta)

                    if abs(error_theta) < 0.1:
                        self.publish_wheel_velocities(bot_id, [0.0,0.0,0.0],100.0 , 80.0 )
                        time.sleep(0.3)
                        bot_data["stack_phase"] = "DROP"
                        return

                    self.publish_wheel_velocities(bot_id, [omega, omega, omega], 100.0, default_arm["elbow"])
                    return
                
                if bot_data["stack_phase"] == "DROP":
                    req = Attach.Request()
                    req.robot_id = bot_id
                    req.flag = False
                    res = Attach.Response()
                    self.attach_callback(req, res)
                    print(res.success)
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],100.0,pick_arm["elbow"])
                    time.sleep(0.5)
                    color = self.crate_colour(crate)

                    if bot_id in self.dropzone_slots[color]:
                        self.dropzone_slots[color].remove(bot_id)
                    
                    bot_data["drop_slot"] = None
                    self.dropzone_count[color] += 1

                    
                    self.dropped_crates.append(crate)

                    if self.zone_lock[color] == bot_id:
                        print(f"Bot {bot_id} released {color} zone")
                        self.zone_lock[color] = None

                    if self.unassigned_crates:
                            left_crate = self.unassigned_crates.pop(0)
                            bot_data["crate_id"] = left_crate
                            bot_data["goal"] = self.crate_positions[left_crate]
                            bot_data["link_attached"] = False 
                            bot_data["link_detach"] = False
                            bot_data["state"] = "APPROACH"
                    else: 
                        bot_data["state"] = "DOCK"
                    del bot_data["stack_phase"]
                    continue
                
            if state == "ZONE_ALIGN":
                print("ZONE ALIGNING")
                goal_x, goal_y, _ = dropzones[color]
                dx = goal_x - bot_x
                dy = goal_y - bot_y 
                angle_to_goal = math.atan2(dy, dx) + math.pi/2
                error_theta = (angle_to_goal - bot_theta)
                error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta)) #normalize
                print("herer error thetha is ",error_theta)
                max_w = 0.4
                min_w = 0.15
                Kp_theta = 0.9
                # abs_error_theta=abs(error_theta)
                omega = Kp_theta * abs(error_theta)
                omega = min(max_w, omega)
                omega = max(min_w, omega)
                omega *=math.copysign(1, error_theta)
                print("ZONE ALIGN ERROR:",error_theta)
                if abs(error_theta) < 0.1:
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],pick_arm["base"],pick_arm["elbow"])
                    time.sleep(0.5)
                    bot_data["state"] = "DETACH"
                    return
                self.publish_wheel_velocities(bot_id,[omega, omega, omega],default_arm["base"]-5,default_arm["elbow"])

            if state == "DETACH":
                req = Attach.Request()
                req.robot_id = bot_id
                req.flag = False
                res = Attach.Response()
                self.attach_callback(req, res)
                print(res.success)
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],pick_arm["base"],pick_arm["elbow"])
                color = self.crate_colour(crate)

                if bot_id in self.dropzone_slots[color]:
                    self.dropzone_slots[color].remove(bot_id)
                
                bot_data["drop_slot"] = None
                self.dropzone_count[color] += 1

                
                self.dropped_crates.append(crate)

                if self.zone_lock[color] == bot_id:
                    print(f"Bot {bot_id} released {color} zone")
                    self.zone_lock[color] = None

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
                error_x = goal_x - bot_x - 40
                error_y = goal_y - bot_y 
                distance = math.sqrt(error_x*error_x + error_y*error_y)

                for id in self.dropped_crates:
                    if id == crate:
                        continue

                    if id in self.crate_positions:
                        cx,cy,_ = self.crate_positions[id]
                        avoid_radius = 260.0 
                        dx_c = bot_x - cx
                        dy_c = bot_y - cy
                        dist = math.sqrt(dx_c*dx_c + dy_c*dy_c)
                        if dist < avoid_radius and dist > 1.0:
                            rep = (avoid_radius - dist) / avoid_radius
                            avoid_gain = 2100.0   # tune
                            error_x += avoid_gain * rep * (dx_c / dist)
                            error_y += avoid_gain * rep * (dy_c / dist)

                if (distance < 25):
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                    bot_data["state"] = "DONE"
                    return
                
                # ---- Bot-to-Bot repulsion (vector-based) ----
                avoid_bot_radius = 300.0     # influence radius
                avoid_bot_gain   = 1900.0    # same scale as crate repulsion, tune separately

                for other_id, other_pose in bot_positions.items():
                    if other_id == bot_id:
                        continue

                    ox, oy, _ = other_pose
                    dx_b = bot_x - ox     # note: vector away from other bot
                    dy_b = bot_y - oy
                    dist_b = math.sqrt(dx_b*dx_b + dy_b*dy_b)

                    if dist_b < avoid_bot_radius and dist_b > 1.0:
                        rep = (avoid_bot_radius - dist_b) / avoid_bot_radius
                        error_x += avoid_bot_gain * rep * (dx_b / dist_b)
                        error_y += avoid_bot_gain * rep * (dy_b / dist_b)

                print("approach distance:",distance,"crate:",crate) 
                self.calculate_publish(error_x, error_y, 0, dt, bot_id, default_arm["base"], default_arm["elbow"])
                print("Dock distance", distance)
            
            if state == "DONE":
                print(f"Bot {name} has completed its task.")
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],default_arm["base"],default_arm["elbow"])
                continue 
    
    def computemidpoint(self):
        coords = []
        for crate_id in self.dropped_crates:
            if crate_id in {12, 30} and crate_id in self.crate_positions:
                cx, cy, _ = self.crate_positions[crate_id]
                coords.append((cx, cy))

        if len(coords) < 2:
            return 1210.0, 1220.0

        (x1, y1), (x2, y2) = coords[0], coords[1]

        midx = (x1 + x2) / 2.0
        midy = (y1 + y2) / 2.0

        # vector from crate1 → crate2
        dx = x2 - x1
        dy = y2 - y1
        # perpendicular vector (normal)
        nx = -dy
        ny = dx
        L = math.hypot(nx, ny)
        nx /= L
        ny /= L
        d = 70.0 
        goal_x = midx + nx * d
        goal_y = midy + ny * d
        return goal_x, goal_y


    def calculate_publish(self, ex, ey, eomega, dt,id, arm, base):
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
        self.publish_wheel_velocities(id,wheel_vel, arm, base)
    
    
    def publish_mqtt(self, topic,payload):
        self.mqtt_client.publish(topic, json.dumps(payload), qos=1,retain=False)

    def publish_wheel_velocities(self, id,wheel_vel=[0.0,0.0,0.0],arm_state=160.0, elbow=80.0):
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
                "arm": {"base": arm_state, "elbow": elbow}
            }
            self.publish_mqtt(f"esp/cmd/{id}", payload)
            print("MQTT CMD:", wheel_vel)


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    controller = HolonomicPIDController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Crucial for fast restarts!
        controller.mqtt_client.loop_stop()
        controller.mqtt_client.disconnect()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()