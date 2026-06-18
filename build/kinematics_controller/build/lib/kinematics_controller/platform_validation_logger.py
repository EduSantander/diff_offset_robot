#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class PlatformValidationLogger(Node):
    def __init__(self):
        super().__init__('platform_validation_logger')
        
        # -----------------------
        # CONFIGURACIÓN
        # -----------------------
        # Nombre del joint que mueve la torreta/plataforma en tu URDF
        self.platform_joint_name = 'top_base_joint' 
        
        self.start_time = self.get_clock().now().nanoseconds
        
        # --- ESTADO ACTUAL ---
        self.phi_dot = 0.0      # Velocidad relativa del motor (rad/s)
        self.theta_c_dot = 0.0  # Velocidad angular del chasis (rad/s)
        
        self.ref_theta_p_dot = 0.0 # Referencia deseada
        
        # --- ALMACENAMIENTO DE DATOS ---
        self.t_data = []
        
        self.w_ref = []    # Deseada (cmd_vel)
        self.w_calc = []   # Calculada (theta_c_dot + phi_dot)
        
        # Debug
        self.joint_found = False

        # --- SUSCRIPTORES ---
        # 1. Comandos deseados (Asumimos que angular.z es la velocidad de plataforma deseada)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # 2. Odometría (Para saber la velocidad de giro del chasis: theta_c_dot)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 3. Velocidad real del motor (phi_dot)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.get_logger().info(f"--- LOGGER VALIDACIÓN PLATAFORMA LISTO ---")
        self.get_logger().info(f"Ecuación: theta_p_dot = theta_c_dot + phi_dot")
        self.get_logger().info("Mueve la plataforma y presiona Ctrl+C para graficar.")

    def get_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    # --- CALLBACKS ---
    def cmd_cb(self, msg):
        # Asumimos que el comando angular.z es la velocidad absoluta que quieres para la plataforma
        self.ref_theta_p_dot = msg.angular.z

    def joint_cb(self, msg):
        try:
            if self.platform_joint_name in msg.name:
                idx = msg.name.index(self.platform_joint_name)
                self.phi_dot = msg.velocity[idx]
                
                if not self.joint_found:
                    self.get_logger().info(f"Joint de plataforma '{self.platform_joint_name}' detectado.")
                    self.joint_found = True
        except ValueError:
            pass

    def odom_cb(self, msg):
        # Usamos el callback de odom para sincronizar y guardar
        t = self.get_time()
        
        # Obtener velocidad angular del chasis (theta_c_dot)
        self.theta_c_dot = msg.twist.twist.angular.z
        
        # --- CÁLCULO DE LA VALIDACIÓN (La Ecuación) ---
        # theta_p_dot = theta_c_dot + phi_dot
        w_real_abs = self.theta_c_dot + self.phi_dot
        
        # Guardar datos
        self.t_data.append(t)
        self.w_ref.append(self.ref_theta_p_dot)
        self.w_calc.append(w_real_abs)

    def plot_results(self):
        if not self.t_data:
            print("No se recibieron datos.")
            return

        print("\nGenerando gráfica de validación de Plataforma...")
        
        plt.figure(figsize=(10, 6))
        plt.title("Validación Velocidad Absoluta Plataforma (theta_p_dot)")
        
        # Referencia
        plt.plot(self.t_data, self.w_ref, 'k--', linewidth=2, label='Ref Deseada (cmd_vel)')
        
        # Calculada Real
        plt.plot(self.t_data, self.w_calc, 'g-', linewidth=2, alpha=0.8, label='Real Calculada (Odom + Joint)')
        
        # (Opcional) Mostrar componentes para depuración
        # plt.plot(self.t_data, [x - y for x, y in zip(self.w_calc, self.w_ref)], 'r:', label='Error')

        plt.ylabel("Velocidad Angular [rad/s]")
        plt.xlabel("Tiempo [s]")
        plt.legend(loc='best')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    logger = PlatformValidationLogger()
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