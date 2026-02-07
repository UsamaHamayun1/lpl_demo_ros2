#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
import cv2
import json
import numpy as np

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, 
                             QPushButton, QHBoxLayout, QFrame, QProgressBar, QGridLayout)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap, QKeyEvent

# --- CONFIGURATION ---
WINDOW_TITLE = "LPL MISSION CONTROL + METRICS"
REFRESH_RATE_MS = 30 

class LPLDashboard(QWidget):
    def __init__(self):
        super().__init__()
        
        # 1. ROS SETUP
        rclpy.init(args=None)
        self.node = Node('lpl_mission_control_qt')
        
        # Subscribers
        self.sub_status = self.node.create_subscription(String, '/lpl/status', self.status_callback, 10)
        self.sub_cam = self.node.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publishers
        self.pub_auth = self.node.create_publisher(Empty, '/lpl/authorize', 10)
        self.pub_input = self.node.create_publisher(Twist, '/cmd_vel_input', 10)
        
        # NEW: Actor Control Publisher
        self.pub_actor = self.node.create_publisher(String, '/actor/mode', 10)

        # State Variables
        self.manual_mode = False
        self.target_lin = 0.0
        self.target_ang = 0.0

        # 2. UI SETUP
        self.init_ui()

        # 3. TIMERS
        self.timer_ros = QTimer(self)
        self.timer_ros.timeout.connect(self.spin_ros)
        self.timer_ros.start(REFRESH_RATE_MS)

        self.timer_drive = QTimer(self)
        self.timer_drive.timeout.connect(self.publish_drive)
        self.timer_drive.start(100) # 10Hz Control Loop

    def init_ui(self):
        self.setWindowTitle(WINDOW_TITLE)
        # Increased height to 850 to fit new buttons
        self.setGeometry(100, 100, 900, 850) 
        self.setStyleSheet("background-color: #121212; color: #e0e0e0;")

        # Main Layout
        layout = QVBoxLayout()
        layout.setSpacing(15)
        layout.setContentsMargins(30, 30, 30, 30)

        # --- HEADER ---
        header = QLabel("ROBOT OPTICAL FEED")
        header.setAlignment(Qt.AlignCenter)
        header.setFont(QFont("Arial", 10, QFont.Bold))
        header.setStyleSheet("color: #888;")
        layout.addWidget(header)

        # --- VIDEO FEED ---
        self.lbl_video = QLabel("NO SIGNAL\n(Check Gazebo)")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: black; border: 2px solid #333; border-radius: 5px;")
        self.lbl_video.setMinimumSize(800, 350)
        self.lbl_video.setSizePolicy(self.lbl_video.sizePolicy().Expanding, self.lbl_video.sizePolicy().Expanding)
        layout.addWidget(self.lbl_video)

        # --- STATUS & TELEMETRY PANEL ---
        status_frame = QFrame()
        status_frame.setStyleSheet("background-color: #1e1e1e; border-radius: 10px; padding: 15px;")
        
        grid_layout = QGridLayout(status_frame)
        grid_layout.setSpacing(10)

        # -- Row 0: Main Headers --
        self.lbl_obj_title = QLabel("TARGET: --")
        self.lbl_obj_title.setFont(QFont("Courier New", 14, QFont.Bold))
        grid_layout.addWidget(self.lbl_obj_title, 0, 0)

        self.lbl_state = QLabel("WAITING")
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setFont(QFont("Arial", 22, QFont.Bold))
        self.lbl_state.setStyleSheet("color: gray;")
        grid_layout.addWidget(self.lbl_state, 0, 1)

        self.lbl_conf_val = QLabel("CONF: 1.00")
        self.lbl_conf_val.setAlignment(Qt.AlignRight)
        self.lbl_conf_val.setFont(QFont("Courier New", 14, QFont.Bold))
        grid_layout.addWidget(self.lbl_conf_val, 0, 2)

        # -- Row 1: Main Confidence Bar --
        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setValue(100)
        self.bar_conf.setTextVisible(False)
        self.bar_conf.setFixedHeight(12)
        self.bar_conf.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
        grid_layout.addWidget(self.bar_conf, 1, 0, 1, 3) 

        # --- METRICS SECTION ---
        
        # -- Row 2: Proximity --
        lbl_prox = QLabel("PROXIMITY (Distance):")
        lbl_prox.setStyleSheet("color: #aaa; font-weight: bold;")
        grid_layout.addWidget(lbl_prox, 2, 0)

        self.bar_prox = QProgressBar()
        self.bar_prox.setRange(0, 100)
        self.bar_prox.setValue(100)
        self.bar_prox.setFixedHeight(8) 
        self.bar_prox.setTextVisible(False)
        self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #00ccff; }")
        grid_layout.addWidget(self.bar_prox, 2, 1, 1, 2) 

        # -- Row 3: Stability --
        lbl_stab = QLabel("STABILITY (Behavior):")
        lbl_stab.setStyleSheet("color: #aaa; font-weight: bold;")
        grid_layout.addWidget(lbl_stab, 3, 0)

        self.bar_stab = QProgressBar()
        self.bar_stab.setRange(0, 100)
        self.bar_stab.setValue(100)
        self.bar_stab.setFixedHeight(8) 
        self.bar_stab.setTextVisible(False)
        self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }")
        grid_layout.addWidget(self.bar_stab, 3, 1, 1, 2) 

        # -- Row 4: Velocity Telemetry --
        vel_label = QLabel("VELOCITY:")
        vel_label.setFont(QFont("Arial", 10, QFont.Bold))
        vel_label.setStyleSheet("color: #666; margin-top: 10px;")
        grid_layout.addWidget(vel_label, 4, 0)

        self.lbl_vel_lin = QLabel("LIN: 0.00 m/s")
        self.lbl_vel_lin.setFont(QFont("Courier New", 12))
        self.lbl_vel_lin.setStyleSheet("color: #00ccff; margin-top: 10px;")
        grid_layout.addWidget(self.lbl_vel_lin, 4, 1)

        self.lbl_vel_ang = QLabel("ANG: 0.00 rad/s")
        self.lbl_vel_ang.setFont(QFont("Courier New", 12))
        self.lbl_vel_ang.setStyleSheet("color: #ffcc00; margin-top: 10px;")
        grid_layout.addWidget(self.lbl_vel_ang, 4, 2, Qt.AlignRight)

        layout.addWidget(status_frame)

        # --- ROBOT CONTROLS ---
        btn_layout = QHBoxLayout()
        
        lbl_instruct = QLabel("CONTROLS:\nWASD to Drive\nSPACE to Override")
        lbl_instruct.setFont(QFont("Arial", 10))
        lbl_instruct.setStyleSheet("color: #666;")
        btn_layout.addWidget(lbl_instruct)
        
        btn_layout.addStretch()

        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setFixedSize(200, 60)
        self.btn_stop.setFont(QFont("Arial", 12, QFont.Bold))
        self.btn_stop.setStyleSheet("background-color: #d32f2f; color: white; border-radius: 5px;")
        self.btn_stop.clicked.connect(lambda: self.set_speed(0.0, 0.0))
        btn_layout.addWidget(self.btn_stop)

        self.btn_auth = QPushButton("TAKE CONTROL\n[SPACE]")
        self.btn_auth.setFixedSize(250, 60)
        self.btn_auth.setFont(QFont("Arial", 14, QFont.Bold))
        self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
        self.btn_auth.setEnabled(False)
        self.btn_auth.clicked.connect(self.send_auth)
        btn_layout.addWidget(self.btn_auth)

        layout.addLayout(btn_layout)

        # --- SEPARATOR ---
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: #333;")
        layout.addWidget(line)

        # --- NEW ACTOR CONTROLS (JITTER / STABLE) ---
        actor_layout = QHBoxLayout()
        
        lbl_actor = QLabel("SIMULATION\nCONTROL:")
        lbl_actor.setFont(QFont("Arial", 10, QFont.Bold))
        lbl_actor.setStyleSheet("color: #888;")
        actor_layout.addWidget(lbl_actor)

        # STABLE BUTTON
        self.btn_stable = QPushButton("STABLE MODE (Normal)")
        self.btn_stable.setFixedSize(250, 50)
        self.btn_stable.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_stable.setStyleSheet("background-color: #007bff; color: white; border-radius: 5px;")
        self.btn_stable.clicked.connect(lambda: self.set_actor_mode("SMOOTH"))
        actor_layout.addWidget(self.btn_stable)

        # JITTER BUTTON
        self.btn_jitter = QPushButton("JITTER MODE (High Variance)")
        self.btn_jitter.setFixedSize(250, 50)
        self.btn_jitter.setFont(QFont("Arial", 11, QFont.Bold))
        self.btn_jitter.setStyleSheet("background-color: #fd7e14; color: white; border-radius: 5px;")
        self.btn_jitter.clicked.connect(lambda: self.set_actor_mode("JITTER"))
        actor_layout.addWidget(self.btn_jitter)
        
        actor_layout.addStretch()
        layout.addLayout(actor_layout)

        self.setLayout(layout)
        self.setFocusPolicy(Qt.StrongFocus)

    # --- LOGIC ---

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def publish_drive(self):
        msg = Twist()
        msg.linear.x = float(self.target_lin)
        msg.angular.z = float(self.target_ang)
        self.pub_input.publish(msg)
        
        self.lbl_vel_lin.setText(f"LIN: {self.target_lin:.2f} m/s")
        self.lbl_vel_ang.setText(f"ANG: {self.target_ang:.2f} rad/s")

    def send_auth(self):
        if self.btn_auth.isEnabled() or self.manual_mode:
            self.pub_auth.publish(Empty())

    def set_speed(self, x, z):
        self.target_lin = x
        self.target_ang = z
    
    # NEW: Function to control the Actor
    def set_actor_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pub_actor.publish(msg)
        print(f"Set Actor Mode: {mode}") # For debugging in terminal

    # --- EVENTS ---
    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()
        # WASD Controls
        if key == Qt.Key_W: self.set_speed(0.3, 0.0)
        elif key == Qt.Key_S: self.set_speed(0.0, 0.0) 
        elif key == Qt.Key_A: self.set_speed(0.0, 0.5) 
        elif key == Qt.Key_D: self.set_speed(0.0, -0.5)
        elif key == Qt.Key_X: self.set_speed(-0.2, 0.0)
        # Auth Control
        elif key == Qt.Key_Space: self.send_auth()
        # Keyboard Shortcuts for Actor
        elif key == Qt.Key_J: self.set_actor_mode("JITTER")
        elif key == Qt.Key_L: self.set_actor_mode("SMOOTH")

    # --- CALLBACKS ---
    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # 1. Main Stats
            conf = float(data.get('confidence', 0.0))
            state = data.get('state', "UNKNOWN")
            obj = data.get('object', "--")
            self.manual_mode = data.get('override', False)
            
            # 2. Detailed Metrics (Prox & Stab)
            # Default to 1.0 (Safe) if missing
            prox = float(data.get('prox', 1.0)) 
            stab = float(data.get('stab', 1.0))

            # UPDATE UI LABELS
            self.lbl_obj_title.setText(f"TARGET: {obj.upper()}")
            self.lbl_state.setText(state)
            self.lbl_conf_val.setText(f"CONF: {conf:.2f}")
            self.bar_conf.setValue(int(conf * 100))
            
            # UPDATE METRIC BARS
            self.bar_prox.setValue(int(prox * 100))
            self.bar_stab.setValue(int(stab * 100))

            # --- DYNAMIC COLORING ---
            
            # Proximity Colors (Cyan -> Red)
            if prox < 0.3: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }") # Red
            elif prox < 0.6: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }") # Yellow
            else: self.bar_prox.setStyleSheet("QProgressBar::chunk { background-color: #00ccff; }") # Cyan
            
            # Stability Colors (Purple -> Red)
            if stab < 0.3: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ff3333; }") # Red
            elif stab < 0.6: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #ffcc00; }") # Yellow
            else: self.bar_stab.setStyleSheet("QProgressBar::chunk { background-color: #aa00ff; }") # Purple

            # Main State Coloring
            if self.manual_mode:
                c = "#00ccff" 
                self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
                self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")
                self.btn_auth.setStyleSheet(f"background-color: {c}; color: black; border-radius: 5px;")
                self.btn_auth.setText("RESUME AUTO\n[SPACE]")
                self.btn_auth.setEnabled(True)

            elif state == "DANGER STOP" or state == "UNSTABLE":
                c = "#ff3333" 
                self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
                self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")
                self.btn_auth.setStyleSheet("background-color: white; color: black; border-radius: 5px;")
                self.btn_auth.setText("TAKE CONTROL\n[SPACE]")
                self.btn_auth.setEnabled(True)

            elif state == "PROCEED":
                c = "#28a745" 
                self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
                self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")
                self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
                self.btn_auth.setText("AUTO ENGAGED")
                self.btn_auth.setEnabled(False)

            else: 
                c = "#ffc107" # Caution
                self.lbl_state.setStyleSheet(f"color: {c}; font-weight: bold;")
                self.bar_conf.setStyleSheet(f"QProgressBar::chunk {{ background-color: {c}; }}")
                self.btn_auth.setStyleSheet("background-color: #444; color: #888; border-radius: 5px;")
                self.btn_auth.setText("CAUTION")
                self.btn_auth.setEnabled(False)

        except Exception:
            pass

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = np_arr.reshape(msg.height, msg.width, -1)
            if 'rgb' in msg.encoding: cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            scaled_pixmap = QPixmap.fromImage(qt_img).scaled(
                self.lbl_video.width(), self.lbl_video.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_video.setPixmap(scaled_pixmap)
        except Exception: pass

    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = LPLDashboard()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()