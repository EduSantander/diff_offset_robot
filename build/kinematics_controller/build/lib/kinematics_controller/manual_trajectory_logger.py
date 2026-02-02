#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np

class ManualTrajectoryLogger(Node):
    def __init__(self):
        super().__init__('manual_trajectory_logger')
        
        # --- ALMACENAMIENTO DE DATOS ---
        # 1. Trayectoria Estimada (Odometría /odom - ROJO)
        self.path_odom_x = []
        self.path_odom_y = []
        
        # 2. Trayectoria Real (Ground Truth /ground_truth/odom - VERDE)
        self.path_gt_x = []
        self.path_gt_y = []

        # --- SUSCRIPTORES ---
        # Escucha lo que calcula tu nodo de cinemática
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Escucha la verdad absoluta de Gazebo (Requiere plugin P3D)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_cb, 10)
        
        self.get_logger().info("--- LOGGER DE TRAYECTORIA (Rojo vs Verde) ---")
        self.get_logger().info("1. Mueve el robot con tu teleop (teclado/joystick).")
        self.get_logger().info("2. Presiona Ctrl+C cuando termines para ver la comparativa.")

    def odom_cb(self, msg):
        self.path_odom_x.append(msg.pose.pose.position.x)
        self.path_odom_y.append(msg.pose.pose.position.y)

    def gt_cb(self, msg):
        self.path_gt_x.append(msg.pose.pose.position.x)
        self.path_gt_y.append(msg.pose.pose.position.y)

    def plot_results(self):
        # Verificación básica para no graficar vacío
        if not self.path_gt_x and not self.path_odom_x:
            print("No se recibieron datos. ¿El robot se movió?")
            return

        print("\nGenerando gráfica comparativa...")
        
        plt.figure(figsize=(10, 8))
        plt.title("VALIDACIÓN: Odometría vs Realidad")
        
        # 1. La realidad (Verde Grueso)
        if self.path_gt_x:
            plt.plot(self.path_gt_x, self.path_gt_y, 'g-', linewidth=3, alpha=0.6, label='Real (Ground Truth)')
            # Punto final real
            plt.plot(self.path_gt_x[-1], self.path_gt_y[-1], 'gx', markersize=10, markeredgewidth=3)
        
        # 2. Lo que el robot cree (Rojo Punteado)
        if self.path_odom_x:
            plt.plot(self.path_odom_x, self.path_odom_y, 'r--', linewidth=2, label='Estimada (Odometría)')
            # Punto final estimado
            plt.plot(self.path_odom_x[-1], self.path_odom_y[-1], 'rx', markersize=10, markeredgewidth=3)

        # Configuración estética
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.legend()
        plt.grid(True)
        plt.axis('equal') # Vital para ver las curvas sin deformación
        
        # Cálculo de Error Final en pantalla
        if self.path_gt_x and self.path_odom_x:
            err_x = self.path_gt_x[-1] - self.path_odom_x[-1]
            err_y = self.path_gt_y[-1] - self.path_odom_y[-1]
            dist_error = np.sqrt(err_x**2 + err_y**2)
            plt.suptitle(f"Error Final de Posición: {dist_error:.4f} m", fontsize=14, color='red')
            print(f"Error final: {dist_error:.4f} metros")

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    logger = ManualTrajectoryLogger()
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