#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import math
import json
import statistics 

class LPLManagerFormula(Node):
    def __init__(self):
        super().__init__('lpl_manager_formula')
        
        self.confidence = 1.0
        self.manual_override = False # <--- NEW FLAG
        self.target_object = "None"
        
        self.vel_history = [] 
        self.history_size = 10 
        
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, 'cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

    def authorize_callback(self, msg):
        # TOGGLE LOGIC:
        # If we are in Auto, switch to Manual.
        # If we are in Manual, switch to Auto.
        self.manual_override = not self.manual_override
        
        if self.manual_override:
            self.get_logger().info("MANUAL OVERRIDE: ENGAGED")
        else:
            self.get_logger().info("MANUAL OVERRIDE: DISENGAGED (Resuming Auto)")

    def calculate_stability_score(self, new_velocity):
        self.vel_history.append(new_velocity)
        if len(self.vel_history) > self.history_size: self.vel_history.pop(0)
        if len(self.vel_history) < 2: return 1.0
        try: variance = statistics.variance(self.vel_history)
        except: return 1.0
        
        jitter_threshold = 0.1
        if variance >= jitter_threshold: return 0.0
        else: return 1.0 - (variance / jitter_threshold)

    def perception_callback(self, msg):
        try:
            # 1. Find Robot & Object
            robot_idx = -1
            possible_names = ['waffle_pi', 'turtlebot3_waffle_pi', 'turtlebot3_burger', 'mobile_base']
            for name in possible_names:
                if name in msg.name:
                    robot_idx = msg.name.index(name)
                    break
            if robot_idx == -1: return 
            robot_pos = msg.pose[robot_idx].position

            closest_dist = 999.0
            closest_name = "None"
            closest_vel_mag = 0.0
            
            for i, name in enumerate(msg.name):
                if name in possible_names or 'ground' in name or 'sun' in name: continue
                p = msg.pose[i].position
                dist = math.sqrt((robot_pos.x - p.x)**2 + (robot_pos.y - p.y)**2)
                if dist < closest_dist: 
                    closest_dist = dist
                    closest_name = name
                    v = msg.twist[i].linear
                    closest_vel_mag = math.sqrt(v.x**2 + v.y**2 + v.z**2)

            self.target_object = closest_name

            # 2. Calculate Formula (ALWAYS RUNNING)
            safe_dist = 2.0
            crit_dist = 0.9 
            if closest_dist >= safe_dist: prox_score = 1.0
            elif closest_dist <= crit_dist: prox_score = 0.0
            else: prox_score = (closest_dist - crit_dist) / (safe_dist - crit_dist)

            stab_score = self.calculate_stability_score(closest_vel_mag)
            
            # RAW CONFIDENCE
            self.confidence = prox_score * stab_score
            if self.confidence < 0.05: self.confidence = 0.0

            # 3. Publish Status
            self.publish_status_to_ui(closest_name)

        except Exception:
            pass

    def control_callback(self, msg):
        safe_msg = Twist()
        safe_msg.angular.z = msg.angular.z 
        velocity_request = msg.linear.x

        if self.manual_override:
            # MODE A: HUMAN CONTROL (Ignore Confidence)
            # Pass exactly what the human pressed on joystick
            safe_msg.linear.x = velocity_request
            safe_msg.angular.z = msg.angular.z 
        else:
            # MODE B: AUTO/LPL CONTROL (Respect Confidence)
            if velocity_request < 0.0:
                safe_msg.linear.x = velocity_request # Reverse always allowed
            else:
                # Multiply speed by confidence
                safe_msg.linear.x = velocity_request * self.confidence

        self.pub_cmd.publish(safe_msg)

    def publish_status_to_ui(self, obs_name):
        # Determine State Label for UI
        if self.manual_override:
            state = "MANUAL"
        elif self.confidence >= 0.8:
            state = "PROCEED"
        elif self.confidence == 0.0:
            state = "PAUSE"
        else:
            state = "CAUTION"

        status_packet = {
            "confidence": self.confidence,
            "state": state,
            "object": obs_name,
            "override": self.manual_override
        }
        self.pub_status.publish(String(data=json.dumps(status_packet)))

def main(args=None):
    rclpy.init(args=args)
    node = LPLManagerFormula()
    rclpy.spin(node)
    rclpy.shutdown()