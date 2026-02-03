#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import math
import json
import statistics
import time
from collections import deque 

# --- AVOIDANCE STATES ---
STATE_CRUISE = 0
STATE_SWERVE_OUT = 1
STATE_PASSING = 2
STATE_MERGE_BACK = 3

class LPLManagerFormula(Node):
    def __init__(self):
        super().__init__('lpl_manager_formula')
        
        # --- CONFIGURATION ---
        self.LPL_ENABLED = True 
        
        # --- STATE VARIABLES ---
        self.confidence = 1.0
        self.smooth_conf = 1.0
        self.last_raw_conf = 1.0 
        
        # Avoidance State Machine
        self.drive_state = STATE_CRUISE
        self.state_start_time = 0
        self.closest_dist = 99.0 
        
        # Stability Buffer
        self.vel_history = deque(maxlen=10) 
        
        # Recovery
        self.recovery_mode = False
        self.recovery_value = 0.0
        
        # --- COMMUNICATION ---
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, '/cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

        self.get_logger().info("--- LPL MANAGER: TUNED FOR STATIC BOX ---")

    def authorize_callback(self, msg):
        if self.confidence < 0.3:
            self.recovery_mode = True
            self.recovery_value = self.confidence 

    def calculate_stability_score(self, new_velocity):
        self.vel_history.append(new_velocity)
        if len(self.vel_history) < 2: return 1.0
        
        try: variance = statistics.variance(self.vel_history)
        except: return 1.0
        
        jitter_threshold = 0.15 
        if variance >= jitter_threshold: return 0.0
        else: return 1.0 - (variance / jitter_threshold)

    def perception_callback(self, msg):
        try:
            # 1. FIND ROBOT
            robot_idx = -1
            possible_names = ['waffle_pi', 'turtlebot3_waffle_pi', 'mobile_base']
            for name in possible_names:
                if name in msg.name:
                    robot_idx = msg.name.index(name)
                    break
            if robot_idx == -1: return 
            
            robot_pos = msg.pose[robot_idx].position
            rx, ry = robot_pos.x, robot_pos.y

            # 2. FIND CLOSEST OBSTACLE
            self.closest_dist = 999.0
            closest_name = "None"
            closest_vel_mag = 0.0
            
            for i, name in enumerate(msg.name):
                if i == robot_idx: continue
                if any(x in name for x in ['ground', 'sun', 'floor']): continue
                
                ox = msg.pose[i].position.x
                oy = msg.pose[i].position.y
                dist = math.sqrt((rx - ox)**2 + (ry - oy)**2)
                
                if dist < self.closest_dist: 
                    self.closest_dist = dist
                    closest_name = name
                    v = msg.twist[i].linear
                    closest_vel_mag = math.sqrt(v.x**2 + v.y**2 + v.z**2)

            # 3. CALCULATE CONFIDENCE
            if self.LPL_ENABLED:
                # --- TUNED VALUES ---
                # We lower crit_dist to 0.3 so the score stays high even when close.
                safe_dist = 2.0 
                crit_dist = 0.5 
                
                if self.closest_dist >= safe_dist: prox_score = 1.0
                elif self.closest_dist <= crit_dist: prox_score = 0.0
                else: prox_score = (self.closest_dist - crit_dist) / (safe_dist - crit_dist)

                stab_score = self.calculate_stability_score(closest_vel_mag)
                
                raw_confidence = prox_score * stab_score
                if raw_confidence < 0.05: raw_confidence = 0.0

                # Panic Stop
                drop = self.last_raw_conf - raw_confidence
                if drop > 0.5:
                    self.smooth_conf = raw_confidence 
                else:
                    alpha = 0.3
                    self.smooth_conf = (1.0 - alpha) * self.smooth_conf + (alpha * raw_confidence)

                self.last_raw_conf = raw_confidence

                if self.recovery_mode:
                    self.recovery_value += 0.05
                    if self.recovery_value >= 1.0:
                        self.recovery_value = 1.0
                        self.recovery_mode = False 
                    self.smooth_conf = max(self.smooth_conf, self.recovery_value)

            else:
                self.smooth_conf = 1.0 

            self.publish_status_to_ui(closest_name)

        except Exception as e:
            self.get_logger().error(f"Perception Error: {e}")

    def control_callback(self, msg):
        final_cmd = Twist()
        current_time = time.time()
        
        if self.drive_state == STATE_CRUISE:
            final_cmd.linear.x = msg.linear.x * 3.5 
            if final_cmd.linear.x > 0.8: final_cmd.linear.x = 0.8 
            final_cmd.angular.z = msg.angular.z
            
            # --- TUNED GATEKEEPER ---
            # Trigger at 1.5m with lower confidence threshold (0.4)
            if self.closest_dist < 2.0 and final_cmd.linear.x > 0:
                if self.smooth_conf > 0.4:
                    self.drive_state = STATE_SWERVE_OUT
                    self.state_start_time = current_time
                    self.get_logger().info(f"OBSTACLE ({self.closest_dist:.1f}m): SWERVING")
                else:
                    # Moving object (Conf=0.0) -> Pass -> Stop
                    pass

        elif self.drive_state == STATE_SWERVE_OUT:
            final_cmd.linear.x = 0.5 
            final_cmd.angular.z = 0.8 
            if current_time - self.state_start_time > 0.8: 
                self.drive_state = STATE_PASSING
                self.state_start_time = current_time

        elif self.drive_state == STATE_PASSING:
            final_cmd.linear.x = 0.8 
            final_cmd.angular.z = 0.0
            if current_time - self.state_start_time > 1.5: 
                self.drive_state = STATE_MERGE_BACK
                self.state_start_time = current_time

        elif self.drive_state == STATE_MERGE_BACK:
            final_cmd.linear.x = 0.5
            final_cmd.angular.z = -0.8
            if current_time - self.state_start_time > 0.8:
                self.drive_state = STATE_CRUISE 
                self.get_logger().info("RESUMING CRUISE")

        final_cmd.linear.x = final_cmd.linear.x * self.smooth_conf
        final_cmd.angular.z = final_cmd.angular.z * self.smooth_conf

        self.pub_cmd.publish(final_cmd)

    def publish_status_to_ui(self, obs_name):
        current_val = self.smooth_conf
        if self.drive_state != STATE_CRUISE: state_label = "AVOIDING"
        elif current_val < 0.4: state_label = "DANGER STOP"
        elif current_val < 0.7: state_label = "CAUTION"
        else: state_label = "PROCEED"

        status_packet = {
            "confidence": round(current_val, 2),
            "state": state_label,
            "object": obs_name,
            "override": self.recovery_mode
        }
        self.pub_status.publish(String(data=json.dumps(status_packet)))

def main(args=None):
    rclpy.init(args=args)
    node = LPLManagerFormula()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()