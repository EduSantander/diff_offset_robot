#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- CONFIGURACIÓN DE TECLAS ---
# w/x: Velocidad en X (Adelante/Atrás)
# a/d: Velocidad en Y (Izquierda/Derecha)
# q/e: Rotación de la Base Superior (Angular Z)
# s/ESPACIO: Parada de emergencia

msg = """
---------------------------------------
CONTROL DE ROBOT DIFF-OFFSET + TORRETA
---------------------------------------
Mover el Chasis:
   q    w    e
   a    s    d
        x

w/x : Aumentar/Disminuir velocidad X (Adelante/Atrás)
a/d : Aumentar/Disminuir velocidad Y (Lateral)
q/e : Girar Base Superior (Izquierda/Derecha)

ESPACIO o 's' : PARADA FORZADA

CTRL-C para salir
"""

# Ajustes de velocidad máximos y pasos de incremento
MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.0
LIN_STEP_SIZE = 0.05
ANG_STEP_SIZE = 0.1

class CustomTeleop(Node):
    def __init__(self):
        super().__init__('custom_teleop_node')
        
        # Publicamos en /cmd_vel que es lo que escucha tu nodo de cinemática
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estado actual de velocidades
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_ang_z = 0.0 # Esto controla tu base superior
        
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_state(self):
        print(f"\rVel X: {self.target_vx:.2f} | Vel Y: {self.target_vy:.2f} | Base Z: {self.target_ang_z:.2f}   ", end="")

    def run(self):
        print(msg)
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # --- Lógica de Control ---
                
                # Eje X (Adelante / Atrás)
                if key == 'w':
                    self.target_vx = min(self.target_vx + LIN_STEP_SIZE, MAX_LIN_VEL)
                elif key == 'x':
                    self.target_vx = max(self.target_vx - LIN_STEP_SIZE, -MAX_LIN_VEL)
                
                # Eje Y (Izquierda / Derecha - Tu cinemática lo permite)
                elif key == 'a':
                    self.target_vy = min(self.target_vy + LIN_STEP_SIZE, MAX_LIN_VEL)
                elif key == 'd':
                    self.target_vy = max(self.target_vy - LIN_STEP_SIZE, -MAX_LIN_VEL)
                
                # Eje Z (Rotación de la Base Superior)
                elif key == 'q':
                    self.target_ang_z = min(self.target_ang_z + ANG_STEP_SIZE, MAX_ANG_VEL)
                elif key == 'e':
                    self.target_ang_z = max(self.target_ang_z - ANG_STEP_SIZE, -MAX_ANG_VEL)
                
                # Parada (Stop)
                elif key == 's' or key == ' ':
                    self.target_vx = 0.0
                    self.target_vy = 0.0
                    self.target_ang_z = 0.0
                    print("\n¡PARADA!")

                # Salir
                elif key == '\x03': # Ctrl+C
                    break

                # Publicar mensaje
                twist = Twist()
                twist.linear.x = float(self.target_vx)
                twist.linear.y = float(self.target_vy)
                twist.angular.z = float(self.target_ang_z)
                
                self.publisher_.publish(twist)
                self.print_state()

        except Exception as e:
            print(e)

        finally:
            # Al salir, enviar comando de parada para que el robot no se quede moviendo
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()