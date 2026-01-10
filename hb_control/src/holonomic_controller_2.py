#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import math
import time
import json
import numpy as np
import paho.mqtt.client as mqtt
from scipy.optimize import linear_sum_assignment
from hb_interfaces.msg import Pose2D, Poses2D, BotCmd, BotCmdArray
from linkattacher_msgs.srv import Attach


# ===================== CONSTANTS =====================

class Params:
    MAX_WHEEL = 600.0
    APPROACH_DIST = 180.0
    ALIGN_DIST = 25.0
    DROP_DIST = 20.0
    DOCK_DIST = 25.0

    ARM_Base = 155.0
    ARM_Elbow = 80.0

    controlLoopFreq = 0.03       #this is the execution frequency of the control loop in seconds


# ===================== ENUMS =====================

class BotState(Enum):
    APPROACH = 1
    ALIGN = 2
    LINK = 3
    DROPZONE = 4
    DETACH = 5
    DOCK = 6
    DONE = 7


# ===================== PID =====================

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / max(dt, 1e-6)
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.limit, min(out, self.limit))


# ===================== ROBOT CLASS =====================

class Robot:
    def __init__(self, robot_id, name, dock, pid_params):
        self.id = robot_id
        self.name = name
        self.dock = dock

        self.state = BotState.APPROACH
        self.pose = None
        self.goal = None
        self.crate_id = None
        self.ir_triggered = False

        self.pid_x = PID(**pid_params["x"])
        self.pid_y = PID(**pid_params["y"])
        self.pid_theta = PID(**pid_params["theta"])

        self.last_time = None
        self.state_start = None


# ===================== MAIN NODE =====================

class HolonomicPIDController(Node):
    #actual ros class which is connected to Node

    def __init__(self):
        super().__init__("holonomic_pid_controller")

        #controlling mqt rate
        self.mqtt_tick = 0
        self.MQTT_DIVIDER = 2   # 33 / 2 ≈ 16.5 Hz

        # ---------- PID Params ----------
        self.pid_params = {
            "x": {"kp": 4.7, "ki": 0.0, "kd": 1.2, "limit": 1.0},
            "y": {"kp": 4.7, "ki": 0.0, "kd": 1.2, "limit": 1.0},
            "theta": {"kp": 3.0, "ki": 0.1, "kd": 0.3, "limit": 2.0},
        }

        # ---------- Robots ----------
        self.bots = {
            0: Robot(0, "crystal", [1218.0, 205.0, 0.0], self.pid_params),
            2: Robot(2, "frostbite", [1568.0, 202.0, 0.0], self.pid_params),
            4: Robot(4, "glacio", [864.0, 204.0, 0.0], self.pid_params),
        }

        # ---------- Drop Zones ----------
        self.dropzones = {
            "red": [1219, 1200, 0],         #D1 zone
            "green": [820, 2000.5, 0],
            "blue": [1616, 2000.5, 0],      #D2 zone
        }

        # ---------- Crates ----------
        self.crate_positions = {}
        self.assigned_crates = set()
        self.unassigned_crates = []

        # ---------- Kinematics ----------
        self.wheel_matrix = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [ math.cos(math.radians(30)),  math.cos(math.radians(150)), 0.0],
            [1.0, 1.0, 1.0]
        ])

        # ---------- ROS ----------
        self.create_subscription(Poses2D, "/bot_pose", self.bot_pose_cb, 10)
        self.create_subscription(Poses2D, "/crate_pose", self.crate_pose_cb, 10)
        self.publisher = self.create_publisher(BotCmdArray, "/bot_cmd", 10)
        self.attach_srv = self.create_service(Attach, "/Attach", self.attach_callback)


        # ---------- MQTT ----------
        self.mqtt_connected = False
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.on_disconnect = self.on_disconnect
        # self.mqtt.connect("localhost", 1883, 60)
        self.mqtt.connect("10.125.32.248", 1883, 60)
        self.mqtt.loop_start()

        # ---------- Timer ----------
        self.timer = self.create_timer(Params.controlLoopFreq, self.control_loop)

    # ================= MQTT =================
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
            self.mqtt_connected = True
            for bid in self.bots:
                client.subscribe(f"esp/sensor/{bid}")
        else:
            self.get_logger.warning("Connection failed, rc =", rc)

    def on_message(self, client, userdata, msg):
        # topic = msg.topic
        bot_id = int(msg.topic.split("/")[-1])
        self.bots[bot_id].ir_triggered = (msg.payload.decode() == "0.00")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("Disconnected from MQTT broker")

    # ================= ROS CALLBACKS =================

    def bot_pose_cb(self, msg):
        for p in msg.poses:
            if p.id in self.bots:
                self.bots[p.id].pose = [p.x, p.y, p.w]

    def crate_pose_cb(self, msg):
        for p in msg.poses:
            self.crate_positions[p.id] = [p.x, p.y, p.w]
        self.assign_crates()

    # ================= ASSIGNMENT =================

    # def assign_crates(self):
    #     for bot in self.bots.values():
    #         if bot.crate_id is not None or bot.pose is None:
    #             continue
    #         # Build list of AVAILABLE crates
    #         available_crates = {
    #             cid: pos for cid, pos in self.crate_positions.items()
    #             if cid not in self.assigned_crates
    #         }
    #         print(f"[DEBUG] Available crates: {available_crates}")
    #         # No crates left
    #         if not available_crates:
    #             return
            
    #         bx, by, _ = bot.pose
        
    #         nearest = min(
    #             self.crate_positions.items(),       #gives [(crate_id, (cx, cy)), ...]
    #             key=lambda c: math.hypot(c[1][0] - bx, c[1][1] - by),
    #             default=None,
    #         )

    #         crate_id, crate_pos = nearest

    #         # Assign
    #         bot.crate_id = crate_id
    #         bot.goal = crate_pos
    #         self.assigned_crates.add(crate_id)

    def assign_crates(self):
        #hungarian method
        # Select free bots with valid pose
        bots = [
            bot for bot in self.bots.values()
            if bot.crate_id is None and bot.pose is not None
        ]

        # Select unassigned crates
        crate_ids = [
            cid for cid in self.crate_positions
            if cid not in self.assigned_crates
        ]

        if not bots or not crate_ids:
            return

        # Build cost (distance) matrix
        cost_matrix = np.zeros((len(bots), len(crate_ids)))

        for i, bot in enumerate(bots):
            bx, by, _ = bot.pose
            for j, cid in enumerate(crate_ids):
                cx, cy = self.crate_positions[cid]
                cost_matrix[i, j] = math.hypot(cx - bx, cy - by)

        # Solve assignment problem
        bot_indices, crate_indices = linear_sum_assignment(cost_matrix)

        # Apply assignments
        for bi, ci in zip(bot_indices, crate_indices):
            bot = bots[bi]
            crate_id = crate_ids[ci]

            bot.crate_id = crate_id
            bot.goal = self.crate_positions[crate_id]
            self.assigned_crates.add(crate_id)

    def crate_colour(self, id):
        a = id%3
        if a == 0:
            color = "red"
        elif a == 1:
            color = "green"
        else:
            color = "blue"
        return color

    # ================= STATE MACHINE =================

    def control_loop(self):
        now = self.get_clock().now()

        for bot in self.bots.values():
            if bot.pose is None or bot.goal is None:
                continue

            if bot.last_time is None:
                bot.last_time = now
                bot.state_start = now
                continue

            dt = (now - bot.last_time).nanoseconds / 1e9
            bot.last_time = now

            if bot.state == BotState.APPROACH:
                self.handle_approach(bot, dt)
            elif bot.state == BotState.ALIGN:
                self.handle_align(bot)
            elif bot.state == BotState.LINK:
                self.handle_link(bot, now)
            elif bot.state == BotState.DROPZONE:
                self.handle_dropzone(bot, dt)
            elif bot.state == BotState.DETACH:
                self.handle_detach(bot)
            elif bot.state == BotState.DOCK:
                self.handle_dock(bot, dt)

    # ================= HANDLERS =================

    def handle_approach(self, bot, dt):
        bx, by, _ = bot.pose
        gx, gy, _ = bot.goal
        dist = math.hypot(gx - bx, gy - by)

        if dist < Params.APPROACH_DIST:
            bot.state = BotState.ALIGN
            bot.state_start = self.get_clock().now()
            bot.pid_x.reset()
            bot.pid_y.reset()
            return

        self.move_pid(bot, gx - bx, gy - by, dt)

    def handle_align(self, bot):
        self.publish_cmd(bot.id, [0.4, 0.4, 0.4])
        if bot.ir_triggered:
            bot.state = BotState.LINK
            bot.state_start = self.get_clock().now()
            self.stop(bot)

    def handle_link(self, bot, now):
        if (now - bot.state_start).nanoseconds / 1e9 > 1.0:
            self.attach(bot.id, True)
            bot.state = BotState.DROPZONE
            bot.goal = self.dropzones[self.crate_colour(bot.crate_id)]

    def handle_dropzone(self, bot, dt):
        bx, by, _ = bot.pose
        gx, gy, _ = bot.goal
        if math.hypot(gx - bx, gy - by) < Params.DROP_DIST:
            bot.state = BotState.DETACH
            self.stop(bot)
            return
        self.move_pid(bot, gx - bx, gy - by, dt)

    def handle_detach(self, bot):
        self.attach(bot.id, False)
        bot.state = BotState.DOCK
        bot.goal = bot.dock

    def handle_dock(self, bot, dt):
        bx, by, _ = bot.pose
        gx, gy, _ = bot.goal
        if math.hypot(gx - bx, gy - by) < Params.DOCK_DIST:
            bot.state = BotState.DONE
            self.stop(bot)
            return
        self.move_pid(bot, gx - bx, gy - by, dt)

    # ================= MOTION =================

    def move_pid(self, bot, ex, ey, dt):
        _, _, theta = bot.pose
        vx = bot.pid_x.compute(ex, dt)
        vy = bot.pid_y.compute(ey, dt)

        x_r = math.cos(theta) * vx + math.sin(theta) * vy
        y_r = -math.sin(theta) * vx + math.cos(theta) * vy

        wheels = np.linalg.pinv(self.wheel_matrix) @ np.array([-x_r, y_r, 0.0])
        wheels /= Params.MAX_WHEEL
        self.publish_cmd(bot.id, wheels.tolist())       #wheels = [m1, m2, m3]

    def stop(self, bot):
        self.publish_cmd(bot.id, [0, 0, 0])

    # ================= IO =================

    def publish_cmd(self, bot_id, wheels):
        self.mqtt_tick += 1

        # ---- ROS ----
        msg = BotCmdArray()
        cmd = BotCmd()
        cmd.id = bot_id
        cmd.m1, cmd.m2, cmd.m3 = wheels
        cmd.base = Params.ARM_Base
        cmd.elbow = Params.ARM_Elbow
        msg.cmds.append(cmd)
        self.publisher.publish(msg)

        # ---- MQTT ----
        if self.mqtt_connected:
            payload = {
                "m1": max(-1.0, min(1.0, float(wheels[0]))),
                "m2": max(-1.0, min(1.0, float(wheels[1]))),
                "m3": max(-1.0, min(1.0, float(wheels[2]))),
                "arm": {
                    "base": Params.ARM_UP,
                    "elbow": Params.ARM_DOWN
                }
            }
            if self.mqtt_connected and self.mqtt_tick % self.MQTT_DIVIDER == 0:
                self.mqtt.publish(
                    f"esp/cmd/{bot_id}",
                    json.dumps(payload),
                    qos=1,
                    retain=False
                )


    def attach(self, bot_id, flag):
        req = Attach.Request()
        req.robot_id = bot_id
        req.flag = flag
        self.attach_srv(req, Attach.Response())


# ===================== MAIN =====================

def main():
    rclpy.init()
    node = HolonomicPIDController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
