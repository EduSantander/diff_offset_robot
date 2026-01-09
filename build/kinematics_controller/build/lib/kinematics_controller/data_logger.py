#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import threading

# Mensajes
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class RobotPerformanceLogger(Node):
    def __init__(self):
        super().__init__('performance_logger')
        
        # --- ALMACENAMIENTO DE DATOS ---
        self.start_time = self.get_clock().now().nanoseconds
        
        # 1. ENTRADAS (Setpoints del Usuario - /cmd_vel)
        self.t_input = []
        self.ref_vx = []
        self.ref_vy = []
        self.ref_theta_p_dot = [] # Velocidad absoluta deseada de la plataforma

        # 2. SALIDAS CALCULADAS (Kinematics Node -> Controladores)
        self.t_calc = []
        self.calc_alpha_L = []
        self.calc_alpha_R = []
        self.calc_phi_dot = []    # Velocidad relativa calculada para el motor

        # 3. REALIDAD (Feedback de Gazebo - /joint_states)
        self.t_real = []
        self.real_alpha_L = []
        self.real_alpha_R = []
        self.real_phi_dot = []

        # --- SUSCRIPTORES ---
        
        # A) Lo que tú pides
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # B) Lo que tu nodo calcula (Salidas a motores)
        self.create_subscription(Float64MultiArray, '/wheel_controller/commands', self.wheels_cmd_cb, 10)
        self.create_subscription(Float64MultiArray, '/platform_controller/commands', self.plat_cmd_cb, 10)
        
        # C) Lo que realmente pasa (Sensores)
        self.create_subscription(JointState, '/joint_states', self.joints_cb, 10)

        self.get_logger().info("--- LOGGER INICIADO: Realiza tus pruebas y presiona Ctrl+C para graficar ---")

    def get_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    # --- CALLBACKS ---
    def cmd_cb(self, msg):
        self.t_input.append(self.get_time())
        self.ref_vx.append(msg.linear.x)
        self.ref_vy.append(msg.linear.y)
        self.ref_theta_p_dot.append(msg.angular.z)

    def wheels_cmd_cb(self, msg):
        # Asumimos que llega [Left, Right]
        self.t_calc.append(self.get_time())
        if len(msg.data) >= 2:
            self.calc_alpha_L.append(msg.data[0])
            self.calc_alpha_R.append(msg.data[1])
        
        # Sincronización simple para phi_dot (si no ha llegado dato nuevo, repetimos el último o ponemos 0)
        if len(self.calc_phi_dot) < len(self.t_calc):
            val = self.calc_phi_dot[-1] if len(self.calc_phi_dot) > 0 else 0.0
            self.calc_phi_dot.append(val)

    def plat_cmd_cb(self, msg):
        # Asumimos que llega [phi_dot]
        if len(self.calc_phi_dot) < len(self.t_calc):
             self.calc_phi_dot.append(msg.data[0])
        elif len(self.calc_phi_dot) == len(self.t_calc):
             self.calc_phi_dot[-1] = msg.data[0] 

    def joints_cb(self, msg):
        try:
            # Buscar índices por nombre (para no depender del orden)
            idx_L = msg.name.index('left_wheel_joint')
            idx_R = msg.name.index('right_wheel_joint')
            idx_P = msg.name.index('top_base_joint')
            
            self.t_real.append(self.get_time())
            self.real_alpha_L.append(msg.velocity[idx_L])
            self.real_alpha_R.append(msg.velocity[idx_R])
            self.real_phi_dot.append(msg.velocity[idx_P])
        except ValueError:
            pass 

    # --- PLOTTING ---
    def plot_results(self):
        if len(self.t_input) == 0:
            print("No se recibieron datos. ¿Ejecutaste la simulación?")
            return

        print("\nGenerando gráficas de desempeño...")
        
        # FIGURA 1: Entradas vs Ruedas Calculadas
        plt.figure(figsize=(12, 8))
        
        plt.subplot(3, 1, 1)
        plt.title("ENTRADA: Velocidades Deseadas (Chasis)")
        plt.plot(self.t_input, self.ref_vx, 'r', label='Vx Deseada')
        plt.plot(self.t_input, self.ref_vy, 'b', label='Vy Deseada')
        plt.ylabel("[m/s]")
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.title("SALIDA CALCULADA: Velocidades de Ruedas")
        if len(self.t_calc) > 0:
            plt.plot(self.t_calc, self.calc_alpha_L, 'g--', label='Alpha L (Calc)')
            plt.plot(self.t_calc, self.calc_alpha_R, 'm--', label='Alpha R (Calc)')
        plt.ylabel("[rad/s]")
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 3)
        plt.title("RELACIÓN TORRETA: Absoluta Deseada vs Motor Relativo")
        plt.plot(self.t_input, self.ref_theta_p_dot, 'k', label='Theta_P_dot (Input)')
        if len(self.t_calc) > 0:
            limit = min(len(self.t_calc), len(self.calc_phi_dot))
            plt.plot(self.t_calc[:limit], self.calc_phi_dot[:limit], 'c--', label='Phi_dot (Motor Calc)')
        plt.ylabel("[rad/s]")
        plt.xlabel("Tiempo [s]")
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()

        # FIGURA 2: Validación de Controladores (Setpoint vs Realidad)
        plt.figure(figsize=(12, 6))
        plt.suptitle("VALIDACIÓN DE CONTROLADORES (Setpoint vs Feedback)")

        # SUBPLOT IZQUIERDO: Ruedas Motrices
        plt.subplot(1, 2, 1)
        plt.title("Ruedas Motrices")
        if len(self.t_calc) > 0 and len(self.t_real) > 0:
            plt.plot(self.t_calc, self.calc_alpha_L, 'r--', label='Ref Izq')
            plt.plot(self.t_real, self.real_alpha_L, 'r-', alpha=0.5, label='Real Izq')
            plt.plot(self.t_calc, self.calc_alpha_R, 'b--', label='Ref Der')
            plt.plot(self.t_real, self.real_alpha_R, 'b-', alpha=0.5, label='Real Der')
        plt.legend()
        plt.ylabel("[rad/s]")  # <--- AGREGADO: Unidad de velocidad angular
        plt.xlabel("Tiempo [s]") # <--- AGREGADO: Unidad de tiempo
        plt.grid(True)

        # SUBPLOT DERECHO: Motor Torreta
        plt.subplot(1, 2, 2)
        plt.title("Motor Torreta")
        if len(self.t_calc) > 0 and len(self.t_real) > 0:
             limit = min(len(self.t_calc), len(self.calc_phi_dot))
             plt.plot(self.t_calc[:limit], self.calc_phi_dot[:limit], 'g--', label='Ref Phi_dot')
             plt.plot(self.t_real, self.real_phi_dot, 'g-', alpha=0.5, label='Real Phi_dot')
        plt.legend()
        plt.ylabel("[rad/s]")  # <--- AGREGADO: Unidad de velocidad angular
        plt.xlabel("Tiempo [s]") # <--- AGREGADO: Unidad de tiempo
        plt.grid(True)

        plt.tight_layout() # Para que no se monten las etiquetas
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    logger = RobotPerformanceLogger()
    
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