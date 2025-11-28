#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

class ClientAGUI(Node):
    def __init__(self):
        super().__init__('gui_client_a')
        self.publisher_ = self.create_publisher(Twist, '/collaborative/position_cmd', 10)
        self.root = tk.Tk()
        self.root.title("Usuario A - Posición (XYZ)")
        self.root.geometry("300x400")
        
        self.create_widgets()
        self.update_ros() # Loop de ROS dentro de Tkinter

    def create_widgets(self):
        ttk.Label(self.root, text="Control de Posición", font=("Arial", 14)).pack(pady=10)
        
        # Botones X
        frame_x = ttk.LabelFrame(self.root, text="Eje X (Adelante/Atras)")
        frame_x.pack(fill="x", padx=10, pady=5)
        ttk.Button(frame_x, text="Adelante (+X)", command=lambda: self.send_cmd(x=1.0)).pack(side="left", expand=True)
        ttk.Button(frame_x, text="Atrás (-X)", command=lambda: self.send_cmd(x=-1.0)).pack(side="left", expand=True)

        # Botones Y
        frame_y = ttk.LabelFrame(self.root, text="Eje Y (Izq/Der)")
        frame_y.pack(fill="x", padx=10, pady=5)
        ttk.Button(frame_y, text="Izquierda (+Y)", command=lambda: self.send_cmd(y=1.0)).pack(side="left", expand=True)
        ttk.Button(frame_y, text="Derecha (-Y)", command=lambda: self.send_cmd(y=-1.0)).pack(side="left", expand=True)

        # Botones Z
        frame_z = ttk.LabelFrame(self.root, text="Eje Z (Arriba/Abajo)")
        frame_z.pack(fill="x", padx=10, pady=5)
        ttk.Button(frame_z, text="Subir (+Z)", command=lambda: self.send_cmd(z=1.0)).pack(side="left", expand=True)
        ttk.Button(frame_z, text="Bajar (-Z)", command=lambda: self.send_cmd(z=-1.0)).pack(side="left", expand=True)

        ttk.Button(self.root, text="DETENER", command=lambda: self.send_cmd()).pack(pady=20, fill="x")

    def send_cmd(self, x=0.0, y=0.0, z=0.0):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        self.publisher_.publish(msg)

    def update_ros(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self.update_ros)

def main():
    rclpy.init()
    gui = ClientAGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()