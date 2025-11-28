#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tkinter as tk
from tkinter import ttk

class ClientBGUI(Node):
    def __init__(self):
        super().__init__('gui_client_b')
        self.pub_orient = self.create_publisher(Twist, '/collaborative/orientation_cmd', 10)
        self.pub_grip = self.create_publisher(Bool, '/collaborative/gripper_cmd', 10)
        
        self.root = tk.Tk()
        self.root.title("Usuario B - Orientación & Gripper")
        self.root.geometry("350x450")
        self.create_widgets()
        self.update_ros()

    def create_widgets(self):
        ttk.Label(self.root, text="Control Muñeca & Pinza", font=("Arial", 14)).pack(pady=10)

        # Gripper
        frame_g = ttk.LabelFrame(self.root, text="Pinza (Gripper)")
        frame_g.pack(fill="x", padx=10, pady=10)
        ttk.Button(frame_g, text="ABRIR", command=lambda: self.send_grip(False)).pack(side="left", expand=True, padx=5)
        ttk.Button(frame_g, text="CERRAR", command=lambda: self.send_grip(True)).pack(side="left", expand=True, padx=5)

        # Rotacion
        frame_rot = ttk.LabelFrame(self.root, text="Rotación Muñeca")
        frame_rot.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Pitch
        ttk.Label(frame_rot, text="Pitch (Arriba/Abajo)").pack()
        ttk.Button(frame_rot, text="▲", command=lambda: self.send_rot(y=1.0)).pack()
        ttk.Button(frame_rot, text="▼", command=lambda: self.send_rot(y=-1.0)).pack()
        
        # Roll/Yaw
        frame_ry = ttk.Frame(frame_rot)
        frame_ry.pack(pady=5)
        ttk.Button(frame_ry, text="Roll ◄", command=lambda: self.send_rot(x=-1.0)).pack(side="left")
        ttk.Button(frame_ry, text="Roll ►", command=lambda: self.send_rot(x=1.0)).pack(side="left")
        ttk.Button(frame_ry, text="Yaw ◄", command=lambda: self.send_rot(z=1.0)).pack(side="left")
        ttk.Button(frame_ry, text="Yaw ►", command=lambda: self.send_rot(z=-1.0)).pack(side="left")

    def send_rot(self, x=0.0, y=0.0, z=0.0):
        msg = Twist()
        msg.angular.x = x
        msg.angular.y = y
        msg.angular.z = z
        self.pub_orient.publish(msg)

    def send_grip(self, close):
        msg = Bool()
        msg.data = close
        self.pub_grip.publish(msg)

    def update_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self.update_ros)

def main():
    rclpy.init()
    gui = ClientBGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()