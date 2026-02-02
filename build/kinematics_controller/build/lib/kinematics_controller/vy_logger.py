#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VyValidationLogger(Node):
    def __init__(self):
        super().__init__('vy_validation_logger')
        
        self.start_time = self.get_clock().now().nanoseconds
        
        # --- ALMACENAMIENTO DE DATOS ---
        self.t_data = []
        self.ref_vy = []   # Lo que pides (Input)
        self.real_vy = []  # Lo que el robot hace (Odometría)
        self.current_ref_vy = 0.0

        # --- SUSCRIPTORES ---
        # 1. Escuchamos tu comando (SetPoint)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # 2. Escuchamos la odometría (Feedback)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.get_logger().info("--- LOGGER Vy LISTO: Mueve el robot (A/D) y presiona Ctrl+C para graficar ---")

    def get_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    def cmd_cb(self, msg):
        self.current_ref_vy = msg.linear.y

    def odom_cb(self, msg):
        t = self.get_time()
        self.t_data.append(t)
        # Velocidad lineal en Y del chasis (frame local del robot)
        self.real_vy.append(msg.twist.twist.linear.y)
        self.ref_vy.append(self.current_ref_vy)

    def plot_results(self):
        if not self.t_data:
            print("No se recibieron datos.")
            return

        print("\nGenerando gráfica...")
        plt.figure(figsize=(10, 6))
        plt.title("VALIDACIÓN: Velocidad Lateral (Vy)")
        plt.plot(self.t_data, self.ref_vy, 'r--', linewidth=2, label='Vy Deseada')
        plt.plot(self.t_data, self.real_vy, 'b-', alpha=0.8, label='Vy Medida (Odom)')
        plt.ylabel("Velocidad [m/s]")
        plt.xlabel("Tiempo [s]")
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    logger = VyValidationLogger()
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