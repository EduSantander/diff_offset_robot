#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import csv

class WheelStepLogger(Node):
    def __init__(self):
        super().__init__('wheel_step_logger')
        
        # --- PUBLICADOR Y SUSCRIPTOR ---
        self.publisher_ = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.start_time = self.get_clock().now().nanoseconds
        
        # --- PARÁMETROS DEL ESCALÓN (LO QUE PUEDES MODIFICAR) ---
        # Como cambiamos a "effort", esto ahora son Newtons-metro (N·m) de Torque puro.
        self.step_amplitude = 5.0 
        self.step_start_time = 2.0  # Segundos en reposo antes del golpe de torque
        
        # --- ALMACENAMIENTO DE DATOS ---
        self.t_data = []
        self.cmd_data = []
        self.wl_data = []
        self.wr_data = []
        
        self.current_cmd = 0.0
        
        # Timer de alta frecuencia (50Hz) para inyectar la señal ininterrumpidamente
        self.timer = self.create_timer(0.02, self.timer_cb)
        
        self.get_logger().info("--- INYECTOR Y LOGGER DE RUEDAS INICIADO ---")
        self.get_logger().info(f"En {self.step_start_time}s se inyectará un torque de {self.step_amplitude} N.m.")
        self.get_logger().info("Deja que el robot acelere y presiona Ctrl+C para guardar el CSV.")

    def get_time(self):
        return (self.get_clock().now().nanoseconds - self.start_time) / 1e9

    def timer_cb(self):
        t = self.get_time()
        
        # Lógica del escalón
        if t >= self.step_start_time:
            self.current_cmd = self.step_amplitude
        else:
            self.current_cmd = 0.0
            
        msg = Float64MultiArray()
        # Mandamos el mismo torque a ambas ruedas: [Izquierda, Derecha]
        msg.data = [self.current_cmd, self.current_cmd]
        self.publisher_.publish(msg)

    def joint_cb(self, msg):
        try:
            idx_l = msg.name.index('left_wheel_joint')
            idx_r = msg.name.index('right_wheel_joint')
            
            vel_l = msg.velocity[idx_l]
            vel_r = msg.velocity[idx_r]
            
            t = self.get_time()
            self.t_data.append(t)
            self.cmd_data.append(self.current_cmd)
            self.wl_data.append(vel_l)
            self.wr_data.append(vel_r)
            
        except ValueError:
            pass 

    def save_to_csv(self):
        if not self.t_data:
            print("\n[ERROR] No se registraron datos de /joint_states.")
            return
            
        filename = 'ruedas_step_response.csv'
        print(f"\n[INFO] Guardando {len(self.t_data)} muestras en {filename}...")
        
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Tiempo_s', 'Torque_Aplicado_Nm', 'Velocidad_Izq_rad_s', 'Velocidad_Der_rad_s'])
            for i in range(len(self.t_data)):
                writer.writerow([self.t_data[i], self.cmd_data[i], self.wl_data[i], self.wr_data[i]])
                
        print("[INFO] ¡Archivo CSV generado con éxito! Listo para importar a MATLAB.")

def main(args=None):
    rclpy.init(args=args)
    node = WheelStepLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Freno automático al presionar Ctrl+C
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0, 0.0]
        node.publisher_.publish(stop_msg)
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()