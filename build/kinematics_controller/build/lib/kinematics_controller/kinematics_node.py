#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import math
import numpy as np

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DiffOffsetKinematics(Node):

    def __init__(self):
        super().__init__('diff_offset_kinematics')

        # -----------------------
        # PARAMETERS
        # -----------------------
        self.R = self.declare_parameter('wheel_radius', 0.055).value
        self.d1 = self.declare_parameter('d1', 0.11225).value
        self.d2 = self.declare_parameter('d2', 0.36).value

        # controller topics
        self.wheel_cmd_topic = self.declare_parameter(
            'wheel_cmd_topic', '/wheel_controller/commands').value

        self.platform_cmd_topic = self.declare_parameter(
            'platform_cmd_topic', '/platform_controller/commands').value

        # frames (antes hardcodeados) y tasa de actualizacion
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame_id = self.declare_parameter('base_frame_id', 'base_link').value
        self.update_rate = self.declare_parameter('update_rate', 100.0).value  # Hz

        # -----------------------
        # INTERNAL STATE
        # -----------------------
        self.x = 0.0
        self.y = 0.0
        self.theta_c = 0.0
        self.theta_p = 0.0

        self.alpha_L = 0.0
        self.alpha_R = 0.0

        # INPUTS DESEADOS
        # vx_cmd_local / vy_cmd_local: tal como llegan por /cmd_vel,
        # en el frame LOCAL del robot (base_link). Ej: vx_cmd_local=0.5
        # significa "0.5 m/s hacia donde apunta el robot ahora mismo".
        self.vx_cmd_local = 0.0
        self.vy_cmd_local = 0.0
        self.theta_p_dot_cmd = 0.0  # yaw rate, no depende de frame

        # vx_des / vy_des: version transformada a frame GLOBAL (odom).
        # Estas son las que realmente necesita compute_kinematics(),
        # ya que la matriz M fue derivada asumiendo velocidades
        # globales del punto de control desplazado (offset d1).
        self.vx_des = 0.0
        self.vy_des = 0.0

        # -----------------------
        # ROS INTERFACES
        # -----------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmdvel_cb, 10)

        self.pub_wheels = self.create_publisher(Float64MultiArray,
                                                self.wheel_cmd_topic, 10)

        self.pub_platform = self.create_publisher(Float64MultiArray,
                                                  self.platform_cmd_topic, 10)

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------
        # TIMER
        # -----------------------
        # NOTA: usamos el reloj de ROS (get_clock) en vez de time.time()
        # para que dt sea consistente con los timestamps que ya se publican
        # (odom, TF) y con use_sim_time si algun dia lo activas.
        self.last_time = self.get_clock().now()
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info("DiffOffsetKinematics reorganizado listo.")

    # ----------------------------------------------------------
    #                   CALLBACKS ROS
    # ----------------------------------------------------------
    def cmdvel_cb(self, msg):
        # cmd_vel llega en frame LOCAL (base_link), por convencion ROS.
        # NO se transforma aqui: theta_c puede cambiar entre callbacks
        # y el timer, asi que la transformacion se hace en update()
        # con el theta_c mas actual posible.
        self.vx_cmd_local = msg.linear.x
        self.vy_cmd_local = msg.linear.y
        self.theta_p_dot_cmd = msg.angular.z

    # ----------------------------------------------------------
    #          0) TRANSFORMACION LOCAL -> GLOBAL
    # ----------------------------------------------------------
    def local_to_global(self, vx_local, vy_local, theta):
        """
        Rota una velocidad expresada en el frame del robot (base_link)
        al frame fijo (odom), usando la orientacion actual del chasis.
        """
        vx_global = vx_local * math.cos(theta) - vy_local * math.sin(theta)
        vy_global = vx_local * math.sin(theta) + vy_local * math.cos(theta)
        return vx_global, vy_global

    # ----------------------------------------------------------
    #          1) CALCULO MATEMATICO (separado)
    # ----------------------------------------------------------
    def compute_kinematics(self):
        """
        Resuelve el sistema M u = b  para obtener:
        u = [alphaL_dot, alphaR_dot, phi_dot]^T
        donde phi_dot es la velocidad relativa del motor de la torreta.
        """
        th = self.theta_c

        R = self.R
        d1 = self.d1
        d2 = self.d2

        # Construccion de la matriz M (3x3)
        a11 = (R*math.cos(th)/2) + (R*d1*math.sin(th)/d2)
        a12 = (R*math.cos(th)/2) - (R*d1*math.sin(th)/d2)
        a21 = (R*math.sin(th)/2) - (R*d1*math.cos(th)/d2)
        a22 = (R*math.sin(th)/2) + (R*d1*math.cos(th)/d2)

        M = np.array([
            [a11, a12, 0],
            [a21, a22, 0],
            [-R/d2, R/d2, 1]
        ], dtype=float)

        # Vector b (Entradas deseadas)
        # b = [vx, vy, theta_p_dot]
        b = np.array([self.vx_des, self.vy_des, self.theta_p_dot_cmd], float)

        try:
            sol = np.linalg.solve(M, b)
            return sol[0], sol[1], sol[2]   # alphaL_dot, alphaR_dot, phi_dot
        except np.linalg.LinAlgError:
            self.get_logger().warn("Sistema cinematico singular")
            return 0.0, 0.0, 0.0

    # ----------------------------------------------------------
    #          2) INTEGRACION DEL ESTADO
    # ----------------------------------------------------------
    def integrate_state(self, alphaL_dot, alphaR_dot, phi_dot, dt):

        # integrar ruedas
        self.alpha_L += alphaL_dot * dt
        self.alpha_R += alphaR_dot * dt

        # velocidad yaw del chasis
        omega_c = (self.R / self.d2) * (alphaR_dot - alphaL_dot)
        self.theta_c += omega_c * dt

        # Calcular velocidad absoluta de la plataforma (theta_p_dot)
        theta_p_dot_real = omega_c + phi_dot

        # integrar plataforma absoluta
        self.theta_p += theta_p_dot_real * dt

        # integrar posicion del robot
        self.x += self.vx_des * dt
        self.y += self.vy_des * dt

        return omega_c, theta_p_dot_real

    # ----------------------------------------------------------
    #          3) PUBLICACION A CONTROLADORES
    # ----------------------------------------------------------
    def publish_commands(self, alphaL_dot, alphaR_dot, phi_dot):
        # ruedas: Float64MultiArray con [left, right]
        wheels = Float64MultiArray()
        wheels.data = [float(alphaL_dot), float(alphaR_dot)]
        self.pub_wheels.publish(wheels)

        # plataforma: Float64MultiArray con [phi_dot]
        msg_platform = Float64MultiArray()
        msg_platform.data = [float(phi_dot)]
        self.pub_platform.publish(msg_platform)

    # ----------------------------------------------------------
    #          4) PUBLICAR JOINT STATES
    # ----------------------------------------------------------
    def publish_joint_states(self, alphaL_dot, alphaR_dot, phi_dot, omega_c):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint", "top_base_joint"]
        phi = self.theta_p - self.theta_c
        js.position = [self.alpha_L, self.alpha_R, phi]
        js.velocity = [
            alphaL_dot,  # Velocidad REAL de rueda izquierda
            alphaR_dot,  # Velocidad REAL de rueda derecha
            phi_dot      # Velocidad relativa del motor de la torreta
        ]
        self.pub_js.publish(js)

    # ----------------------------------------------------------
    #          5) PUBLICAR ODOMETRY
    # ----------------------------------------------------------
    def publish_odometry(self, omega_c):
        current_time = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.theta_c)

        # --- A) PUBLICAR MENSAJE ODOMETRY ---
        od = Odometry()
        od.header.stamp = current_time
        od.header.frame_id = self.odom_frame_id
        od.child_frame_id = self.base_frame_id

        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = q

        # IMPORTANTE: por convencion de nav_msgs/Odometry (REP 103/105),
        # "pose" va en header.frame_id (global/odom) pero "twist" va en
        # child_frame_id (local/base_link). Por eso aqui usamos las
        # componentes LOCALES (tal como llegaron por /cmd_vel), no las
        # globales que usa la cinematica interna.
        od.twist.twist.linear.x = self.vx_cmd_local
        od.twist.twist.linear.y = self.vy_cmd_local
        od.twist.twist.angular.z = omega_c

        self.pub_odom.publish(od)

        # --- B) PUBLICAR TRANSFORMADA TF ---
        t = TransformStamped()

        # Cabecera
        t.header.stamp = current_time
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id

        # Traslacion (Posicion calculada)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotacion (Cuaternion calculado)
        t.transform.rotation = q

        # Enviar!
        self.tf_broadcaster.sendTransform(t)

    # ----------------------------------------------------------
    #          6) LOOP PRINCIPAL
    # ----------------------------------------------------------
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_time = now

        # 0. Transformar el comando LOCAL (cmd_vel) a GLOBAL (odom),
        #    usando el theta_c actual. A partir de aqui, vx_des/vy_des
        #    ya estan en el frame que compute_kinematics() necesita.
        self.vx_des, self.vy_des = self.local_to_global(
            self.vx_cmd_local, self.vy_cmd_local, self.theta_c)

        # 1. Calcular: Obtenemos las velocidades de rueda (alphas)
        alphaL_dot, alphaR_dot, phi_dot = self.compute_kinematics()

        # 2. Integrar: Obtenemos velocidad de chasis (omega_c)
        omega_c, theta_p_dot_real = self.integrate_state(alphaL_dot, alphaR_dot, phi_dot, dt)

        # 3. Publicar Comandos
        self.publish_commands(alphaL_dot, alphaR_dot, phi_dot)

        # 4. Publicar Joint States (reactivado: necesario para que
        #    robot_state_publisher arme el resto del arbol TF a partir del URDF)
        self.publish_joint_states(alphaL_dot, alphaR_dot, phi_dot, omega_c)

        # 5. Publicar Odometria
        self.publish_odometry(omega_c)

    # ----------------------------------------------------------
    def euler_to_quaternion(self, roll, pitch, yaw):
        q = Quaternion()
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        q.w = cr*cp*cy + sr*sp*sy
        q.x = sr*cp*cy - cr*sp*sy
        q.y = cr*sp*cy + sr*cp*sy
        q.z = cr*cp*sy - sr*sp*cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = DiffOffsetKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()