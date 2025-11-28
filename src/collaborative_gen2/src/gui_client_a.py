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
        
        # Estado actual del comando
        self.current_cmd = Twist()
        self.sending = False

        # Configurar Ventana
        self.root = tk.Tk()
        self.root.title("Usuario A - Posición (XYZ)")
        self.root.geometry("350x450")
        
        self.create_widgets()
        self.update_ros() # Loop principal

    def create_widgets(self):
        ttk.Label(self.root, text="MANTENGA PRESIONADO PARA MOVER", font=("Arial", 10, "bold")).pack(pady=10)
        
        # --- EJE X ---
        fx = ttk.LabelFrame(self.root, text="Eje X (Adelante/Atras)")
        fx.pack(fill="x", padx=10, pady=5)
        self.make_button(fx, "Adelante (W)", 1.0, 'x')
        self.make_button(fx, "Atrás (S)", -1.0, 'x')

        # --- EJE Y ---
        fy = ttk.LabelFrame(self.root, text="Eje Y (Izq/Der)")
        fy.pack(fill="x", padx=10, pady=5)
        self.make_button(fy, "Izquierda (A)", 1.0, 'y')
        self.make_button(fy, "Derecha (D)", -1.0, 'y')

        # --- EJE Z ---
        fz = ttk.LabelFrame(self.root, text="Eje Z (Arriba/Abajo)")
        fz.pack(fill="x", padx=10, pady=5)
        self.make_button(fz, "Subir (Q)", 1.0, 'z')
        self.make_button(fz, "Bajar (E)", -1.0, 'z')

        # Status
        self.lbl_status = ttk.Label(self.root, text="Estado: Detenido", foreground="red")
        self.lbl_status.pack(pady=20)

    def make_button(self, parent, text, val, axis):
        btn = ttk.Button(parent, text=text)
        btn.pack(side="left", expand=True, padx=5, pady=5)
        # Eventos: Al presionar y al soltar
        btn.bind('<ButtonPress-1>', lambda event: self.start_move(val, axis))
        btn.bind('<ButtonRelease-1>', lambda event: self.stop_move())
        return btn

    def start_move(self, val, axis):
        self.current_cmd = Twist() # Reiniciar
        if axis == 'x': self.current_cmd.linear.x = val
        elif axis == 'y': self.current_cmd.linear.y = val
        elif axis == 'z': self.current_cmd.linear.z = val
        
        self.sending = True
        self.lbl_status.config(text=f"Moviendo en {axis.upper()}...", foreground="green")

    def stop_move(self):
        self.sending = False
        self.current_cmd = Twist() # Todo a 0
        self.publisher_.publish(self.current_cmd) # Enviar parada inmediata
        self.lbl_status.config(text="Estado: Detenido", foreground="red")

    def update_ros(self):
        # Si el boton esta presionado, enviamos comando continuamente
        if self.sending:
            self.publisher_.publish(self.current_cmd)
        
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(50, self.update_ros) # 20Hz refresco

def main():
    rclpy.init()
    gui = ClientAGUI()
    gui.root.mainloop()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()