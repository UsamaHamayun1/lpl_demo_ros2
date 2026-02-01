#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
import math
import json

class LPLManagerSimple(Node):
    def __init__(self):
        super().__init__('lpl_manager_formula')
        
        self.confidence = 1.0
        self.manual_override = False
        self.target_object = "None"
        
        self.sub_models = self.create_subscription(ModelStates, '/gazebo/model_states', self.perception_callback, 10)
        self.sub_input = self.create_subscription(Twist, '/cmd_vel_input', self.control_callback, 10)
        self.sub_auth = self.create_subscription(Empty, '/lpl/authorize', self.authorize_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/lpl/status', 10)

        print("--- LPL MANAGER: SINGLE ACTOR DEMO LOADED ---")

    def authorize_callback(self, msg):
        self.manual_override = not self.manual_override

    def perception_callback(self, msg):
        try:
            # 1. FIND ROBOT
            robot_idx = -1
            if 'turtlebot3_waffle_pi' in msg.name: robot_idx = msg.name.index('turtlebot3_waffle_pi')
            elif 'waffle_pi' in msg.name: robot_idx = msg.name.index('waffle_pi')
            if robot_idx == -1: return

            rx = msg.pose[robot_idx].position.x
            ry = msg.pose[robot_idx].position.y

            closest_dist = 999.0
            closest_name = "None"
            
            # 2. FIND ACTOR
            for i, name in enumerate(msg.name):
                if i == robot_idx: continue
                if any(x in name for x in ['ground', 'sun']): continue
                
                ox = msg.pose[i].position.x
                oy = msg.pose[i].position.y
                dist = math.sqrt((rx - ox)**2 + (ry - oy)**2)
                
                if dist < closest_dist:
                    closest_dist = dist
                    closest_name = name

            self.target_object = closest_name

            # 3. CALCULATE CONFIDENCE (Distance Based)
            # Far (> 3.0m) = 1.0
            # Close (< 1.0m) = 0.0 (STOP)
            if closest_dist > 3.0:
                self.confidence = 1.0
            elif closest_dist < 1.0:
                self.confidence = 0.0
            else:
                # Linear drop between 3.0 and 1.0
                self.confidence = (closest_dist - 1.0) / 2.0

            # 4. DETERMINE STATE LABEL
            if self.manual_override:
                state_label = "MANUAL"
            elif self.confidence == 0.0:
                state_label = "UNSTABLE" # Acts as PAUSE
            elif self.confidence < 0.8:
                state_label = "AVOIDING"
            else:
                state_label = "PROCEED"

            status_packet = {
                "confidence": round(self.confidence, 2),
                "state": state_label,
                "object": closest_name,
                "override": self.manual_override
            }
            self.pub_status.publish(String(data=json.dumps(status_packet)))

        except Exception:
            pass

    def control_callback(self, msg):
        final_cmd = Twist()
        final_cmd.angular.z = msg.angular.z 
        
        if self.manual_override:
            final_cmd.linear.x = msg.linear.x
        else:
            if msg.linear.x > 0:
                final_cmd.linear.x = msg.linear.x * self.confidence
            else:
                final_cmd.linear.x = msg.linear.x 

        self.pub_cmd.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LPLManagerSimple()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()