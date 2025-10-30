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
        if abs(value) < 50 and abs(value) > 5 and self.kp < 0.4:
            value = math.copysign(60,value)
        print(f"[PID] error={error:.3f}, dt={dt:.3f}, kd_error={kd_error:.3f}, output={value:.3f}")
        return value
    
    def reset(self):
        print("[PID] Resetting PID controller")
        self.integral = 0.0
        self.prev_error = 0.0


class HolonomicPIDController(Node):
    def __init__(self):
        super().__init__('holonomic_pid_controller')
        print("[INIT] Initializing HolonomicPIDController")
        
        self.goal_limit = {
            'x': 5.0,
            'y':5.0,
            'theta':0.09
        }

        # ---------------- Goal Definitions ----------------
        self.d1 = [1219,1219,0]
        self.d2 = [820,2017.5,0]
        self.d3= [1616,2017.5,0]
        self.dock_crystal = [1218.0,205.0,0.0]
        
        self.crate_positions = {}
        self.assigned_crates = set()
        self.unassigned_queue = []
        self.crate_assignment_done = False

        self.array = np.array([
            [-math.sin(math.radians(30)), -math.sin(math.radians(150)), 1.0],
            [math.cos(math.radians(30)), math.cos(math.radians(150)), 0.0],
            [0.175, 0.175, 0.175] 
        ])

        self.max_vel = 1.0
        
        # ---------------- PID Parameters ----------------
        self.pid_params = {
            'x': {'kp': 0.3, 'ki': 0.00, 'kd': 0.07, 'max_out': self.max_vel},
            'y': {'kp': 0.3, 'ki': 0.00, 'kd': 0.07, 'max_out': self.max_vel},
            'theta': {'kp': 1.3, 'ki': 0.00, 'kd': 0.5, 'max_out': self.max_vel * 2}
        }
        
        self.bots = {
            0: {
                "state": "ALIGN",
                "name": "crystal",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "link_attached": False,
                "link_detach": False,
                "dock":[1218.0,205.0,0.0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now()
            },
            2: { 
                "state": "ALIGN",
                "name": "frostbite",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "link_attached": False,
                "link_detach": False,
                "dock":[1568.0,202.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now() 
            },
            4: { 
                "state": "ALIGN",
                "name": "glacio",
                "pose": None,
                "goal": None,
                "crate_id": None,
                "link_attached": False,
                "link_detach": False,
                "dock":[864.0,204.0,0],
                "pid_x": PID(**self.pid_params['x']),
                "pid_y": PID(**self.pid_params['y']),
                "pid_theta": PID(**self.pid_params['theta']),
                "last_time": self.get_clock().now()
            }
        }

        print(f"[INIT] Configured {len(self.bots)} bots: {[self.bots[bid]['name'] for bid in self.bots.keys()]}")

        # ---------------- ROS 2 Publishers & Subscribers ----------------
        self.bot_sub = self.create_subscription(Poses2D,'/bot_pose', self.bot_pose_cb,10)
        self.crate_sub = self.create_subscription(Poses2D,'/crate_pose', self.crate_pose_cb,10)
        print("[INIT] Subscribed to /bot_pose and /crate_pose")

        #SERVICES
        self.attach_cli = self.create_client(AttachLink, '/attach_link')
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[SERVICE] Waiting for /attach_link service...')
        print("[INIT] Connected to /attach_link service")
        self.Attach_req = AttachLink.Request()

        self.detach_cli = self.create_client(DetachLink, '/detach_link')
        while not self.detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[SERVICE] Waiting for /detach_link service...')
        print("[INIT] Connected to /detach_link service")
        self.Detach_req = DetachLink.Request()

        self.publisher = self.create_publisher(BotCmdArray, '/bot_cmd', 10)
        print("[INIT] Publisher created for /bot_cmd")
        
        # ---------------- Timer for Control Loop ----------------
        self.timer = self.create_timer(0.03, self.control_cb)
        print("[INIT] Control timer started (30ms)")

    # ---------------- Subscriber Callback ----------------
    def bot_pose_cb(self, msg):
        for pose in msg.poses:
            if pose.id in self.bots:
                self.bots[pose.id]["pose"] = [pose.x,pose.y,pose.w]
                # Reduced verbosity - comment out for less spam
                # print(f"[BOT_POSE] Bot {self.bots[pose.id]['name']} (id={pose.id}): x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.w:.3f}")

    def crate_pose_cb(self, msg):
        print(f"[CRATE_POSE] Received {len(msg.poses)} crate poses")
        for pose in msg.poses:
            crate_id = pose.id
            self.crate_positions[crate_id] = [pose.x, pose.y, pose.w]
            print(f"[CRATE_POSE] Crate {crate_id}: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.w:.3f}")

            # Update goal if already assigned
            for bot_id, bot_data in self.bots.items():
                if bot_data["crate_id"] == crate_id:
                    bot_data["goal"] = [pose.x, pose.y, pose.w]
                    print(f"[CRATE_POSE] Updated goal for bot {bot_data['name']} tracking crate {crate_id}")

        # Try assigning crates if not done
        if not self.crate_assignment_done and len(self.crate_positions) > 0:
            print(f"[CRATE_POSE] Starting crate assignment with {len(self.crate_positions)} crates")
            self.assign_crates_to_bots()
            self.crate_assignment_done = True

    def attach(self,botname,crate):
        print(f"[ATTACH] Attaching crate {crate} to bot {botname}")
        self.Attach_req.data = json.dumps({
                "model1_name": f"hb_{botname}",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{crate}"
            })
        self.future = self.attach_cli.call_async(self.Attach_req)
        print(f"[ATTACH] Attach request sent for {botname} and crate {crate}")

    def detach(self,botname,crate):
        print(f"[DETACH] Detaching crate {crate} from bot {botname}")
        self.Detach_req.data = json.dumps({
                "model1_name": f"hb_{botname}",
                "link1_name": "arm_link_2",
                "model2_name": f"crate_{self.crate_colour(crate)}_{crate}",
                "link2_name": f"box_link_{crate}"
            })
        self.future = self.detach_cli.call_async(self.Detach_req)
        print(f"[DETACH] Detach request sent for {botname} and crate {crate}")
    
    def crate_colour(self,id):
        a = id%3
        color = {0: "red", 1: "green", 2: "blue"}[a]
        return color
    
    def assign_crates_to_bots(self):
        print("[ASSIGN] Starting crate assignment")
        assigned = set()
        bot_positions = {
            bot_id: bot_data["pose"]
            for bot_id, bot_data in self.bots.items()
            if bot_data["pose"] is not None
        }

        if not bot_positions:
            self.get_logger().info("[ASSIGN] No bot poses available yet for assignment.")
            return

        print(f"[ASSIGN] Bot positions: {bot_positions}")
        available_crates = {
            cid: cpose for cid, cpose in self.crate_positions.items() if cid not in self.assigned_crates
        }
        print(f"[ASSIGN] Available crates: {list(available_crates.keys())}")

        for bot_id, bot_pose in bot_positions.items():
            if not available_crates:
                print("[ASSIGN] No more available crates")
                break

            # Compute nearest crate to this bot
            bx, by, _ = bot_pose
            nearest_crate, nearest_dist = None, float('inf')
            for cid, (cx, cy, _) in available_crates.items():
                dist = math.sqrt((cx - bx)**2 + (cy - by)**2)
                print(f"[ASSIGN] Bot {self.bots[bot_id]['name']} to crate {cid}: distance={dist:.2f}")
                if dist < nearest_dist:
                    nearest_crate, nearest_dist = cid, dist

            if nearest_crate is not None:
                # Assign
                self.bots[bot_id]["crate_id"] = nearest_crate
                self.bots[bot_id]["goal"] = available_crates[nearest_crate]
                assigned.add(nearest_crate)
                del available_crates[nearest_crate]

                self.get_logger().info(
                    f"[ASSIGN] ✓ Assigned crate {nearest_crate} to bot {self.bots[bot_id]['name']} (id {bot_id}) at distance {nearest_dist:.2f}"
                )

        # Record remaining crates
        remaining = set(self.crate_positions.keys()) - assigned
        self.unassigned_queue = list(remaining)
        if remaining:
            self.get_logger().info(f"[ASSIGN] Unassigned crates waiting: {remaining}")
        
        self.assigned_crates |= assigned
        print(f"[ASSIGN] Assignment complete. Assigned: {assigned}, Unassigned queue: {self.unassigned_queue}")

    # ---------------- Control Loop ----------------
    def control_cb(self):
        for bot_id,bot_data in self.bots.items():
            name = bot_data["name"]
            state = bot_data["state"]
            botPose = bot_data["pose"]
            goalPose = bot_data["goal"]
            crate = bot_data["crate_id"]
            last_time = bot_data["last_time"]
            lnk_attached = bot_data["link_attached"]
            lnk_detach = bot_data["link_detach"]
            dock = bot_data["dock"]
            self.pid_x = bot_data["pid_x"]
            self.pid_y = bot_data["pid_y"]
            self.pid_theta = bot_data["pid_theta"]

            if(botPose is None):
                # print(f"[CONTROL] Bot {name} (id={bot_id}): No pose yet, skipping")
                continue  # CRITICAL FIX: Use continue instead of return
            
            if(goalPose is None):
                # print(f"[CONTROL] Bot {name} (id={bot_id}): No goal yet, skipping")
                continue  # CRITICAL FIX: Use continue instead of return
            
            # Time delta
            now = self.get_clock().now()
            dt = (now - last_time).nanoseconds / 1e9
            if dt <= 0:
                continue  # CRITICAL FIX: Use continue instead of return
            bot_data["last_time"] = now  # CRITICAL FIX: Update in bot_data dictionary
            
            print(f"\n[CONTROL] === Bot {name} (id={bot_id}) | State: {state} | Crate: {crate} ===")
            
            #Current pose
            bot_x, bot_y, bot_theta = botPose
            goal_x, goal_y, goal_theta = goalPose
            dx = goal_x - bot_x
            dy = goal_y - bot_y 
            angle_to_goal = math.atan2(dy, dx) - math.pi/2
            
            print(f"[CONTROL] Bot pose: ({bot_x:.2f}, {bot_y:.2f}, {bot_theta:.3f})")
            print(f"[CONTROL] Goal pose: ({goal_x:.2f}, {goal_y:.2f}, {goal_theta:.3f})")

            if state == "ALIGN":
                error_theta = -(angle_to_goal - bot_theta)
                error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
                print(f"[ALIGN] angle_to_goal={angle_to_goal:.3f}, error_theta={error_theta:.3f}, bot_theta={bot_theta:.3f}")
                self.calculate_publish(0,0,error_theta,dt,bot_id)
                if (abs(error_theta)< self.goal_limit['theta']):
                    print(f"[ALIGN] ✓ Alignment complete! Switching to APPROACH")
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0])
                    bot_data["state"] = "APPROACH"
                    continue  # CRITICAL FIX: Use continue to process next bot
                
            if state == "APPROACH":
                error_x = goal_x - bot_x 
                error_y = goal_y - bot_y
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
                e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
                print(f"[APPROACH] error=({error_x:.2f}, {error_y:.2f}), distance={distance:.2f}")
                print(f"[APPROACH] robot_frame_error=({e_x_robot:.2f}, {e_y_robot:.2f})")
                self.calculate_publish(e_x_robot,e_y_robot,0,dt,bot_id)
                if (distance < 145):
                    print(f"[APPROACH] ✓ Reached crate! Switching to LINK")
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],90.0)
                    bot_data["state"] = "LINK"
                    continue  # CRITICAL FIX: Use continue to process next bot
                
            if state == "LINK":
                print(f"[LINK] Attaching link, attached={lnk_attached}")
                if not lnk_attached:
                    self.attach(f"{name}",crate)
                    bot_data["link_attached"] = True
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],45.0)
                bot_data["state"] = "DROPZONE"
                print(f"[LINK] ✓ Link attached! Switching to DROPZONE")
                continue  # CRITICAL FIX: Use continue to process next bot
            
            if state== "DROPZONE":
                dropzones = {"red": self.d1, "green": self.d2, "blue": self.d3}
                goal_x, goal_y, goal_theta = dropzones[self.crate_colour(crate)]
                print(f"[DROPZONE] Target: {self.crate_colour(crate)} zone at ({goal_x:.2f}, {goal_y:.2f})")
                error_x = goal_x - bot_x 
                error_y = goal_y - bot_y
                angle_to_goal = math.atan2(error_x, error_y) - math.pi/2
                error_theta = -(angle_to_goal - bot_theta)
                error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
                e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
                self.calculate_publish(e_x_robot,e_y_robot,error_theta,dt,bot_id)
                print(f"[DROPZONE] error=({error_x:.2f}, {error_y:.2f}), distance={distance:.2f}")
                if (distance < 50):
                    print(f"[DROPZONE] ✓ Reached dropzone! Switching to DETACH")
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],45.0)
                    bot_data["state"] = "DETACH"
                    continue  # CRITICAL FIX: Use continue to process next bot
                
            if state == "DETACH":
                print(f"[DETACH] Detaching link, detached={lnk_detach}")
                if not lnk_detach:
                    self.detach(f"{name}",crate)
                    bot_data["link_detach"] = True
                self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],45.0)
                if self.unassigned_queue:
                    next_crate = self.unassigned_queue.pop(0)
                    bot_data["crate_id"] = next_crate
                    bot_data["goal"] = self.crate_positions[next_crate]
                    bot_data["link_attached"] = False  # CRITICAL FIX: Reset flags
                    bot_data["link_detach"] = False    # CRITICAL FIX: Reset flags
                    bot_data["state"] = "ALIGN"
                    self.get_logger().info(f"[DETACH] ✓ Reassigned bot {name} to crate {next_crate}")
                else: 
                    print(f"[DETACH] ✓ No more crates. Switching to DOCK")
                    bot_data["state"] = "DOCK"
                continue  # CRITICAL FIX: Use continue to process next bot

            if state == "DOCK":
                goal_x,goal_y,goal_theta = dock
                print(f"[DOCK] Heading to dock at ({goal_x:.2f}, {goal_y:.2f})")
                error_x = goal_x - bot_x 
                error_y = goal_y - bot_y
                distance = math.sqrt(error_x*error_x + error_y*error_y)
                e_x_robot =  math.cos(bot_theta)*error_x + math.sin(bot_theta)*error_y
                e_y_robot = -math.sin(bot_theta)*error_x + math.cos(bot_theta)*error_y
                self.calculate_publish(e_x_robot,e_y_robot,0,dt,bot_id)
                print(f"[DOCK] error=({error_x:.2f}, {error_y:.2f}), distance={distance:.2f}")
                if (distance < 25):
                    print(f"[DOCK] ✓ Reached dock! Switching to DONE")
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.pid_theta.reset()
                    self.publish_wheel_velocities(bot_id,[0.0,0.0,0.0],0.0,0.0)
                    bot_data["state"] = "DONE"
                    continue  # CRITICAL FIX: Use continue to process next bot
            
            if state == "DONE":
                print(f"[DONE] Bot {name} has completed all tasks")
                continue  # CRITICAL FIX: Use continue to process next bot

    def calculate_publish(self,ex,ey,eomega,dt,id):
        print(f"[CALC] Input errors: ex={ex:.3f}, ey={ey:.3f}, eomega={eomega:.3f}")
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
        print(f"[CALC] PID outputs: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}")
        print(f"[CALC] Wheel speeds: s1={s1:.3f}, s2={s2:.3f}, s3={s3:.3f}")

        self.publish_wheel_velocities(id,velocities,90.0)

    # ---------------- Publisher ----------------
    def publish_wheel_velocities(self,id,wheel_vel=[0.0,0.0,0.0],arm_state = 0.0, elbow = 90.0):
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
        print(f"[PUBLISH] Bot {id}: m1={wheel_vel[0]:.2f}, m2={wheel_vel[1]:.2f}, m3={wheel_vel[2]:.2f}, arm={arm_state:.2f}, elbow={elbow:.2f}")
    
    def convert_to_pi(self,angle):
        radian = math.radians(angle) 
        to_pi = math.atan2(math.sin(radian), math.cos(radian))
        return to_pi


# ---------------------- Main Function -------------------------------------
def main(args=None):
    rclpy.init(args=args)
    print("[MAIN] ROS 2 initialized")
    controller = HolonomicPIDController()
    print("[MAIN] Starting controller spin...")
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
    print("[MAIN] Controller shutdown complete")


if __name__ == '__main__':
    main()