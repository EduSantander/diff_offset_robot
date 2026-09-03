#!/usr/bin/env python3
"""
platform_orientation_controller
--------------------------------
Controla la orientacion ABSOLUTA de la plataforma superior (theta_p).
VERSIÓN CON TF2: Elimina la deriva (drift) leyendo la orientación 
real del chasis desde el sistema de coordenadas global (ej. 'map').

Logica de control:
    theta_p_dot = Kp*error + Ki*integral(error) - omega_c_real
"""
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def wrap_to_pi(angle):
    """Envuelve un angulo a [-pi, pi] -- error angular mas corto."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PlatformOrientationController(Node):

    def __init__(self):
        super().__init__('platform_orientation_controller')

        # R y d2 los seguimos usando SOLO para obtener omega_c (velocidad de giro instantánea)
        self.R = self.declare_parameter('wheel_radius', 0.055).value
        self.d2 = self.declare_parameter('d2', 0.36).value

        self.Kp_theta_p = self.declare_parameter('Kp_theta_p', 1.5).value
        self.Ki_theta_p = self.declare_parameter('Ki_theta_p', 0.4).value
        self.integral_max = self.declare_parameter('integral_max', 1.0).value
        self.max_phi_dot = self.declare_parameter('max_phi_dot', 3.0).value
        self.platform_cmd_topic = self.declare_parameter(
            'platform_cmd_topic', '/platform_controller/commands').value

        # NUEVO: Marcos de referencia (Frames)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # El marco global por defecto será 'odom', pero cambiará a 'map' si Nav2 lo indica
        self.global_frame = 'odom' 

        self.theta_c = 0.0        # Ahora vendrá de TF2, no de integración ciega
        self.omega_c = 0.0        
        self.phi = 0.0            
        self.theta_p_target = None  
        self._integral = 0.0
        self._last_time = self.get_clock().now()
        self._log_timer = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)

        self.pub_platform = self.create_publisher(
            Float64MultiArray, self.platform_cmd_topic, 10)
        self.pub_debug = self.create_publisher(String, '/platform_debug', 10)

        self.get_logger().info("platform_orientation_controller con TF2 (Cero Deriva) LISTO.")

    # ----------------------------------------------------------
    #                   CALLBACKS ROS
    # ----------------------------------------------------------
    def joint_states_cb(self, msg):
        """
        Calculamos dt y obtenemos las posiciones/velocidades relativas.
        Ya NO integramos theta_c aquí.
        """
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        try:
            iL = msg.name.index('left_wheel_joint')
            iR = msg.name.index('right_wheel_joint')
            alphaL_dot = msg.velocity[iL]
            alphaR_dot = msg.velocity[iR]
            # Mantenemos omega_c de los motores porque es instantáneo (Feedforward sin latencia)
            self.omega_c = (self.R / self.d2) * (alphaR_dot - alphaL_dot)
        except (ValueError, IndexError):
            pass 

        try:
            iP = msg.name.index('top_base_joint')
            self.phi = msg.position[iP]
        except (ValueError, IndexError):
            pass

        self.control_step(dt)

    def goal_pose_cb(self, msg):
        """
        Recibe el objetivo y ADAPTA el marco global.
        Si la flecha de RViz fue puesta en el mapa, usaremos el frame 'map'.
        """
        self.global_frame = msg.header.frame_id
        
        q = msg.pose.orientation
        self.theta_p_target = 2.0 * math.atan2(q.z, q.w)
        
        self.get_logger().info(
            f"Nuevo theta_p objetivo en frame '{self.global_frame}': "
            f"{math.degrees(self.theta_p_target):.1f} grados"
        )

    # ----------------------------------------------------------
    #                   LAZO DE CONTROL
    # ----------------------------------------------------------
    def control_step(self, dt):
        """
        Usa TF2 para conocer la rotacion exacta del chasis en el mundo real.
        """
        if dt <= 0.0:
            return

        # 1. PREGUNTAR A TF2 DÓNDE ESTAMOS REALMENTE
        try:
            # time=0 significa "dame el dato más reciente"
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, 
                self.base_frame, 
                rclpy.time.Time()
            )
            q = trans.transform.rotation
            self.theta_c = 2.0 * math.atan2(q.z, q.w)
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Si TF2 falla (ej. todavía no inicia el mapa), abortar este ciclo
            # pero mantener cancelación de giro si es posible.
            if self.theta_p_target is None:
                msg = Float64MultiArray()
                msg.data = [float(-self.omega_c)]
                self.pub_platform.publish(msg)
            return

        # 2. CALCULAR ORIENTACIÓN ABSOLUTA (Exacta)
        theta_p = self.theta_c + self.phi

        # 3. CONTROL
        if self.theta_p_target is None:
            phi_dot_cmd = -self.omega_c
            self._integral = 0.0  
        else:
            error = wrap_to_pi(self.theta_p_target - theta_p)

            # Log para terminal secundaria
            self._log_timer += dt
            if self._log_timer >= 1.0:
                debug_msg = (
                    f"🌍 Frame: {self.global_frame} | "
                    f"🎯 TARGET: {math.degrees(self.theta_p_target):.1f}° | "
                    f"🤖 ACTUAL: {math.degrees(theta_p):.1f}° | "
                    f"⚠️ ERROR: {math.degrees(error):.1f}°"
                )
                msg_str = String()
                msg_str.data = debug_msg
                self.pub_debug.publish(msg_str)
                self._log_timer = 0.0

            # Integral
            self._integral += error * dt
            self._integral = max(-self.integral_max, min(self.integral_max, self._integral))

            # Ecuación final
            phi_dot_cmd = (self.Kp_theta_p * error
                           + self.Ki_theta_p * self._integral
                           - self.omega_c)
            phi_dot_cmd = max(-self.max_phi_dot, min(self.max_phi_dot, phi_dot_cmd))

        # 4. ENVIAR COMANDO AL MOTOR
        msg = Float64MultiArray()
        msg.data = [float(phi_dot_cmd)]
        self.pub_platform.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformOrientationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()