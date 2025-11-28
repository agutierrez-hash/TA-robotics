#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

class ClientBGUI(Node):
    def __init__(self):
        super().__init__('gui_client_b')
        
        # Publicadores
        self.pub_orient = self.create_publisher(Twist, '/collaborative/orientation_cmd', 10)
        # AHORA EL GRIPPER ES TWIST TAMBIEN
        self.pub_grip = self.create_publisher(Twist, '/collaborative/gripper_continuous_cmd', 10)
        
        self.current_cmd = Twist()
        self.current_grip_cmd = Twist()
        self.sending_rot = False
        self.sending_grip = False

        self.root = tk.Tk()
        self.root.title("Usuario B - Muñeca y Pinza")
        self.root.geometry("400x550")
        self.style = ttk.Style()
        self.style.configure("Bold.TButton", font=("Arial", 10, "bold"))
        
        self.create_widgets()
        self.update_ros()

    def create_widgets(self):
        ttk.Label(self.root, text="CONTROL CONTINUO", font=("Arial", 16, "bold")).pack(pady=15)

        # --- GRIPPER CONTINUO ---
        frame_g = ttk.LabelFrame(self.root, text="🖐️ Pinza (Mantener para mover)")
        frame_g.pack(fill="x", padx=15, pady=5)
        
        # Abrir (velocidad negativa) / Cerrar (velocidad positiva)
        self.make_hold_button(frame_g, "ABRIR (1)", -1.0, 'g', side="left")
        self.make_hold_button(frame_g, "CERRAR (2)", 1.0, 'g', side="left")

        # --- ORIENTACIÓN ---
        frame_rot = ttk.LabelFrame(self.root, text="🔄 Muñeca")
        frame_rot.pack(fill="both", expand=True, padx=15, pady=10)

        # Pitch
        self.make_hold_button(frame_rot, "▲ Arriba (I)", 1.0, 'y')
        self.make_hold_button(frame_rot, "▼ Abajo (K)", -1.0, 'y')
        
        ttk.Separator(frame_rot, orient='horizontal').pack(fill='x', pady=5)

        # Roll/Yaw
        f_ry = ttk.Frame(frame_rot)
        f_ry.pack(fill="x")
        self.make_hold_button(f_ry, "Roll ◄ (J)", -1.0, 'x', side="left")
        self.make_hold_button(f_ry, "Roll ► (L)", 1.0, 'x', side="left")
        
        f_yaw = ttk.Frame(frame_rot)
        f_yaw.pack(fill="x")
        self.make_hold_button(f_yaw, "Yaw ◄ (U)", 1.0, 'z', side="left")
        self.make_hold_button(f_yaw, "Yaw ► (O)", -1.0, 'z', side="left")

        self.lbl_status = ttk.Label(self.root, text="Listo", foreground="gray")
        self.lbl_status.pack(pady=10)

    def make_hold_button(self, parent, text, val, mode, side="top"):
        btn = ttk.Button(parent, text=text, style="Bold.TButton")
        btn.pack(side=side, expand=True, fill="x", padx=5, pady=2)
        btn.bind('<ButtonPress-1>', lambda e: self.start(val, mode))
        btn.bind('<ButtonRelease-1>', lambda e: self.stop(mode))
        return btn

    def start(self, val, mode):
        if mode == 'g': # Gripper
            self.current_grip_cmd.linear.x = val
            self.sending_grip = True
            self.lbl_status.config(text="Moviendo Pinza...", foreground="green")
        else: # Rotacion
            self.current_cmd = Twist()
            if mode == 'x': self.current_cmd.angular.x = val
            if mode == 'y': self.current_cmd.angular.y = val
            if mode == 'z': self.current_cmd.angular.z = val
            self.sending_rot = True
            self.lbl_status.config(text="Rotando...", foreground="blue")

    def stop(self, mode):
        if mode == 'g':
            self.sending_grip = False
            self.current_grip_cmd = Twist()
        else:
            self.sending_rot = False
            self.current_cmd = Twist()
        self.lbl_status.config(text="Detenido", foreground="gray")

    def update_ros(self):
        if self.sending_rot:
            self.pub_orient.publish(self.current_cmd)
        if self.sending_grip:
            self.pub_grip.publish(self.current_grip_cmd)
        
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self.update_ros)

def main():
    rclpy.init()
    gui = ClientBGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()