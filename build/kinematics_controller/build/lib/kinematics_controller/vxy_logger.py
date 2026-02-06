#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class VxyValidationLogger(Node):
    def __init__(self):
        super().__init__('vxy_validation_logger')
        
        # -----------------------
        # PARÁMETROS DEL ROBOT (¡Verifica que sean los tuyos!)
        # -----------------------
        self.R = 0.055       # Radio de rueda
        self.d1 = 0.11225    # Distancia Offset
        self.d2 = 0.36       # Distancia entre ruedas (Track Width)
        
        self.start_time = self.get_clock().now().nanoseconds
        
        # --- ESTADO ACTUAL ---
        self.wl = 0.0        # Velocidad rueda izq (rad/s)
        self.wr = 0.0        # Velocidad rueda der (rad/s)
        self.theta_c = 0.0   # Ángulo de dirección actual (rad)
        
        # Referencias actuales
        self.ref_vx = 0.0
        self.ref_vy = 0.0
        
        # --- ALMACENAMIENTO DE DATOS ---
        self.t_data = []
        
        # Datos Vx
        self.vx_ref = []    # 1. Deseada (Input cmd_vel)
        self.vx_odom = []   # 2. Odometría (Tu Nodo)
        self.vx_calc = []   # 3. Calculada Rigurosa (Joints + Theta)
        
        # Datos Vy
        self.vy_ref = []
        self.vy_odom = []
        self.vy_calc = []
        
        # Debug
        self.joints_detected = False

        # --- SUSCRIPTORES ---
        # 1. Comandos deseados
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        # 2. Odometría (para obtener Vx/Vy teóricos y el ángulo theta_c)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # 3. Velocidades reales de las ruedas
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.get_logger().info(f"--- LOGGER RIGUROSO Vx/Vy LISTO ---")
        self.get_logger().info(f"Usando ecuaciones completas con theta_c.")
        self.get_logger().info("Mueve el robot y presiona Ctrl+C para graficar.")

    def get_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    # --- CALLBACKS ---
    def cmd_cb(self, msg):
        self.ref_vx = msg.linear.x
        self.ref_vy = msg.linear.y

    def joint_cb(self, msg):
        if len(msg.velocity) == 0: return
        
        if not self.joints_detected:
            self.get_logger().info(f"Joints detectados: {msg.name}")
            self.joints_detected = True

        try:
            for i, name in enumerate(msg.name):
                if 'left' in name and 'wheel' in name:
                    self.wl = msg.velocity[i]
                if 'right' in name and 'wheel' in name:
                    self.wr = msg.velocity[i]
        except ValueError:
            pass

    def odom_cb(self, msg):
        # Usamos el callback de odom como reloj principal para guardar datos
        t = self.get_time()
        self.t_data.append(t)
        
        # 1. Guardar Referencias
        self.vx_ref.append(self.ref_vx)
        self.vy_ref.append(self.ref_vy)
        
        # 2. Guardar Odometría del Nodo
        # Asumimos que el twist de odom está en el marco local del robot
        self.vx_odom.append(msg.twist.twist.linear.x)
        self.vy_odom.append(msg.twist.twist.linear.y)
        
        # --- EXTRAER THETA_C DE LA ODOMETRÍA ---
        # La orientación viene en cuaternión, necesitamos el ángulo Yaw (theta_c)
        q = msg.pose.pose.orientation
        self.theta_c = self.quaternion_to_yaw(q)
        
        # --- 3. CÁLCULO RIGUROSO (Cinemática Directa Completa) ---
        # Términos intermedios basados en las ruedas
        A = (self.R / 2.0) * (self.wr + self.wl)
        B = (self.R * self.d1 / self.d2) * (self.wr - self.wl)
        
        sin_th = math.sin(self.theta_c)
        cos_th = math.cos(self.theta_c)
        
        # Ecuaciones completas rotadas
        vx_manual = A * cos_th - B * sin_th
        vy_manual = A * sin_th + B * cos_th
        
        self.vx_calc.append(vx_manual)
        self.vy_calc.append(vy_manual)

    # Utilidad para convertir cuaternión a yaw
    def quaternion_to_yaw(self, q):
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def plot_results(self):
        if not self.t_data:
            print("No se recibieron datos.")
            return

        print("\nGenerando gráfica comparativa Vx/Vy...")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f"Validación Cinemática Rigurosa (R={self.R}, d1={self.d1})", fontsize=14)

        # --- GRÁFICA Vx ---
        ax1.set_title("Velocidad Longitudinal Local (Vx)")
        ax1.plot(self.t_data, self.vx_ref, 'k--', linewidth=1.5, label='Ref (cmd_vel)')
        ax1.plot(self.t_data, self.vx_odom, 'r-', linewidth=3, alpha=0.5, label='Odom (Tu Nodo)')
        ax1.plot(self.t_data, self.vx_calc, 'g:', linewidth=2, label='Calc Rigurosa (Joints+Theta)')
        ax1.set_ylabel("Vx [m/s]")
        ax1.legend(loc='upper right')
        ax1.grid(True)
        
        # --- GRÁFICA Vy ---
        ax2.set_title("Velocidad Lateral Local (Vy)")
        ax2.plot(self.t_data, self.vy_ref, 'k--', linewidth=1.5, label='Ref (cmd_vel)')
        ax2.plot(self.t_data, self.vy_odom, 'r-', linewidth=3, alpha=0.5, label='Odom (Tu Nodo)')
        ax2.plot(self.t_data, self.vy_calc, 'g:', linewidth=2, label='Calc Rigurosa (Joints+Theta)')
        ax2.set_ylabel("Vy [m/s]")
        ax2.set_xlabel("Tiempo [s]")
        ax2.legend(loc='upper right')
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    logger = VxyValidationLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.plot_results()
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()