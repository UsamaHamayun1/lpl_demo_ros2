#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
import cv2
import json
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import threading
import numpy as np

# --- CONFIGURATION ---
WINDOW_TITLE = "LPL MISSION CONTROL"
BG_COLOR = "#0f0f0f"
TEXT_COLOR = "#e0e0e0"

class LPLDashboard(Node):
    def __init__(self, root):
        super().__init__('lpl_dashboard')
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1000x800")
        self.root.configure(bg=BG_COLOR)

        # 1. ROS SUBSCRIBERS
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.sub_status = self.create_subscription(String, '/lpl/status', self.status_callback, 10)
        
        # 2. ROS PUBLISHERS
        self.pub_auth = self.create_publisher(Empty, '/lpl/authorize', 10)
        self.pub_input = self.create_publisher(Twist, 'cmd_vel_input', 10)

        # 3. STATE VARIABLES
        self.var_state = tk.StringVar(value="WAITING...")
        self.var_obj = tk.StringVar(value="--")
        self.var_conf = tk.StringVar(value="1.00")
        self.manual_mode = False
        
        # 4. JOYSTICK VARIABLES
        self.target_lin = 0.0
        self.target_ang = 0.0

        # 5. BUILD UI
        self.setup_ui()
        
        # 6. START CONTROL LOOP (10Hz)
        self.create_timer(0.1, self.publish_control)

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')

        # --- CAMERA FEED ---
        self.cam_label = tk.Label(self.root, text="[ WAITING FOR VIDEO ]", bg="black", fg="#333", width=100, height=25)
        self.cam_label.pack(pady=10, fill="both", expand=True)

        # --- DATA PANEL ---
        data_frame = tk.Frame(self.root, bg=BG_COLOR)
        data_frame.pack(fill="x", padx=20, pady=5)

        # Left: Target
        tk.Label(data_frame, text="TARGET OBSTACLE", font=("Arial", 8), bg=BG_COLOR, fg="#666").grid(row=0, column=0, sticky="w")
        tk.Label(data_frame, textvariable=self.var_obj, font=("Courier", 18, "bold"), bg=BG_COLOR, fg="white").grid(row=1, column=0, sticky="w")

        # Center: State
        self.lbl_state = tk.Label(data_frame, textvariable=self.var_state, font=("Arial", 24, "bold"), bg=BG_COLOR, fg="#555")
        self.lbl_state.place(relx=0.5, rely=0.5, anchor="center")

        # Right: Scalar Confidence
        tk.Label(data_frame, text="CONFIDENCE", font=("Arial", 8), bg=BG_COLOR, fg="#666").place(relx=1.0, y=0, anchor="ne")
        self.lbl_conf = tk.Label(data_frame, textvariable=self.var_conf, font=("Consolas", 36, "bold"), bg=BG_COLOR, fg="#00ff00")
        self.lbl_conf.place(relx=1.0, y=20, anchor="ne")

        # --- BAR GRAPH ---
        self.canvas = tk.Canvas(self.root, height=20, bg="#222", highlightthickness=0)
        self.canvas.pack(fill="x", padx=20, pady=10)
        self.bar = self.canvas.create_rectangle(0, 0, 0, 20, fill="#00ff00", width=0)

        # --- CONTROLS ---
        ctrl_frame = tk.Frame(self.root, bg="#1a1a1a", pady=10)
        ctrl_frame.pack(fill="x", padx=20, pady=10)

        # D-Pad
        pad = tk.Frame(ctrl_frame, bg="#1a1a1a")
        pad.pack(side="left", padx=20)
        btn_opts = {"font":("Arial", 10, "bold"), "bg":"#333", "fg":"white", "width":4, "height":1}
        
        tk.Button(pad, text="▲", command=lambda: self.move(0.2, 0.0), **btn_opts).grid(row=0, column=1)
        tk.Button(pad, text="◄", command=lambda: self.move(0.0, 0.5), **btn_opts).grid(row=1, column=0)
        tk.Button(pad, text="■", command=lambda: self.move(0.0, 0.0), bg="#c00", fg="white", width=4).grid(row=1, column=1)
        tk.Button(pad, text="►", command=lambda: self.move(0.0, -0.5), **btn_opts).grid(row=1, column=2)
        tk.Button(pad, text="▼", command=lambda: self.move(-0.2, 0.0), **btn_opts).grid(row=2, column=1)

        # Authorize Button
        self.btn_auth = tk.Button(ctrl_frame, text="TAKE CONTROL [SPACE]", command=self.authorize, 
                                  font=("Arial", 16, "bold"), bg="#333", fg="#555", state="disabled", height=2)
        self.btn_auth.pack(side="right", fill="both", expand=True, padx=(30, 0))

        # Keybinds
        self.root.bind('<space>', lambda e: self.authorize())
        self.root.bind('<w>', lambda e: self.move(0.2, 0.0))
        self.root.bind('<s>', lambda e: self.move(0.0, 0.0))
        self.root.bind('<a>', lambda e: self.move(0.0, 0.5))
        self.root.bind('<d>', lambda e: self.move(0.0, -0.5))

    def move(self, x, z):
        self.target_lin = x
        self.target_ang = z

    def publish_control(self):
        msg = Twist()
        msg.linear.x = float(self.target_lin)
        msg.angular.z = float(self.target_ang)
        self.pub_input.publish(msg)

    def authorize(self):
        # We allow authorization if paused OR if we are already in manual mode (to toggle back)
        if self.btn_auth['state'] == 'normal' or self.manual_mode:
            self.pub_auth.publish(Empty())

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            conf = float(data.get('confidence', 0.0))
            state = data.get('state', "UNKNOWN")
            obj = data.get('object', "--")
            self.manual_mode = data.get('override', False)

            # Update Variables
            self.var_obj.set(obj.upper())
            self.var_state.set(state)
            self.var_conf.set(f"{conf:.2f}")

            # Update Bar Width
            w = self.canvas.winfo_width()
            self.canvas.coords(self.bar, 0, 0, w * conf, 20)

            # --- COLOR LOGIC ---
            if self.manual_mode:
                c = "#00ccff" # Cyan (Manual)
                self.btn_auth.config(state="normal", bg=c, fg="black", text="RESUME AUTO [SPACE]")
            elif state == "PAUSE":
                c = "#ff3333" # Red (Danger)
                self.btn_auth.config(state="normal", bg="white", fg="black", text="TAKE CONTROL [SPACE]")
            elif state == "PROCEED":
                c = "#00ff00" # Green
                self.btn_auth.config(state="disabled", bg="#222", fg="#555", text="AUTO ENGAGED")
            else:
                c = "#ffaa00" # Yellow
                self.btn_auth.config(state="disabled", bg="#222", fg="#555", text="CAUTION")

            self.lbl_state.config(fg=c)
            self.lbl_conf.config(fg=c)
            self.canvas.itemconfig(self.bar, fill=c)

        except Exception as e:
            print(f"Status Error: {e}")

    def image_callback(self, msg):
        try:
            # MANUAL NUMPY DECODING (NO CV_BRIDGE)
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = np_arr.reshape(msg.height, msg.width, -1)
            
            # RGB vs BGR check
            if 'rgb' in msg.encoding:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            
            # Resize
            cv_img = cv2.resize(cv_img, (800, 450))
            
            # Convert to TK
            img = ImageTk.PhotoImage(PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)))
            self.cam_label.configure(image=img)
            self.cam_label.image = img
        except Exception:
            pass

def main():
    rclpy.init()
    root = tk.Tk()
    app = LPLDashboard(root)
    t = threading.Thread(target=lambda: rclpy.spin(app), daemon=True)
    t.start()
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()