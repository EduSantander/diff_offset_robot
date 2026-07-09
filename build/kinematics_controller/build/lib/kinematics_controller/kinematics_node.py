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

        # IMPORTANTE: en Gazebo con ros2_control + joint_state_broadcaster,
        # /joint_states YA lo publica el broadcaster con los valores REALES
        # simulados de left_wheel_joint, right_wheel_joint y top_base_joint.
        # Si este nodo tambien publica esos mismos joints (con el modelo
        # cinematico, no la fisica real), quedan DOS publishers en el mismo
        # topico con los mismos nombres -> robot_state_publisher recibe
        # valores en conflicto sin ninguna fusion, causando temblores/saltos
        # en el TF. Por eso el default es False en simulacion.
        # Ponlo en True solo si corres en hardware real sin encoders/broadcaster.
        self.publish_model_joint_states = self.declare_parameter(
            'publish_model_joint_states', False).value

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

        # Velocidades REALES medidas (vienen de /joint_states, publicadas
        # por joint_state_broadcaster con la fisica real de Gazebo). Se
        # usan para INTEGRAR el estado (odometria), en vez de creer que
        # el robot ejecuto exactamente lo comandado. Empiezan en 0 y se
        # actualizan en joint_states_cb() apenas llega el primer mensaje.
        self.alphaL_dot_real = 0.0
        self.alphaR_dot_real = 0.0
        self.phi_dot_real = 0.0

        # -----------------------
        # ROS INTERFACES
        # -----------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmdvel_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)

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

    def joint_states_cb(self, msg):
        """
        Lee las velocidades REALES simuladas (publicadas por
        joint_state_broadcaster) de las ruedas y de la torreta.
        Estas son las que se usan para integrar odometria -- no las
        que este nodo calculo/comando.
        """
        try:
            iL = msg.name.index("left_wheel_joint")
            iR = msg.name.index("right_wheel_joint")
            self.alphaL_dot_real = msg.velocity[iL]
            self.alphaR_dot_real = msg.velocity[iR]
        except (ValueError, IndexError):
            # el mensaje no trae esos joints (ej. viene de otra fuente
            # como el plugin de lidar/caster) -> lo ignoramos
            pass

        try:
            iP = msg.name.index("top_base_joint")
            self.phi_dot_real = msg.velocity[iP]
        except (ValueError, IndexError):
            pass

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

    def global_to_local(self, vx_global, vy_global, theta):
        """Inversa de local_to_global (rotacion por -theta)."""
        vx_local = vx_global * math.cos(theta) + vy_global * math.sin(theta)
        vy_local = -vx_global * math.sin(theta) + vy_global * math.cos(theta)
        return vx_local, vy_local

    def _build_M(self, theta):
        """
        Construye la matriz M (3x3) que relaciona [alphaL_dot, alphaR_dot,
        phi_dot] con [vx, vy, theta_p_dot] en el frame global, para el
        angulo de chasis 'theta' dado. Se usa tanto para resolver la
        cinematica inversa (compute_kinematics) como para evaluar la
        cinematica directa (forward_point_velocity), sin duplicar la
        formula del offset.
        """
        R = self.R
        d1 = self.d1
        d2 = self.d2

        a11 = (R*math.cos(theta)/2) + (R*d1*math.sin(theta)/d2)
        a12 = (R*math.cos(theta)/2) - (R*d1*math.sin(theta)/d2)
        a21 = (R*math.sin(theta)/2) - (R*d1*math.cos(theta)/d2)
        a22 = (R*math.sin(theta)/2) + (R*d1*math.cos(theta)/d2)

        return np.array([
            [a11, a12, 0],
            [a21, a22, 0],
            [-R/d2, R/d2, 1]
        ], dtype=float)

    # ----------------------------------------------------------
    #          1) CALCULO MATEMATICO (separado)
    # ----------------------------------------------------------
    def compute_kinematics(self):
        """
        Resuelve el sistema M u = b  para obtener:
        u = [alphaL_dot, alphaR_dot, phi_dot]^T
        donde phi_dot es la velocidad relativa del motor de la torreta.
        """
        M = self._build_M(self.theta_c)

        # Vector b (Entradas deseadas)
        # b = [vx, vy, theta_p_dot]
        b = np.array([self.vx_des, self.vy_des, self.theta_p_dot_cmd], float)

        try:
            sol = np.linalg.solve(M, b)
            return sol[0], sol[1], sol[2]   # alphaL_dot, alphaR_dot, phi_dot
        except np.linalg.LinAlgError:
            self.get_logger().warn("Sistema cinematico singular")
            return 0.0, 0.0, 0.0

    def forward_point_velocity(self, alphaL_dot, alphaR_dot, phi_dot):
        """
        Cinematica DIRECTA: dado un [alphaL_dot, alphaR_dot, phi_dot] real
        (medido), devuelve [vx, vy, theta_p_dot] real en el frame global.
        Es exactamente M @ u -- la misma matriz que compute_kinematics()
        invierte, pero aqui se usa para adelante en vez de resolver.
        """
        M = self._build_M(self.theta_c)
        u = np.array([alphaL_dot, alphaR_dot, phi_dot], float)
        vx, vy, theta_p_dot = M @ u
        return vx, vy, theta_p_dot

    # ----------------------------------------------------------
    #          2) INTEGRACION DEL ESTADO
    # ----------------------------------------------------------
    def integrate_state(self, alphaL_dot_real, alphaR_dot_real, phi_dot_real, dt):
        """
        Integra el estado (theta_c, theta_p, x, y) usando velocidades
        REALES medidas via /joint_states -- no las comandadas. Esto cierra
        el lazo: si el robot no ejecuta exactamente lo comandado (inercia,
        friccion, limites del PID), la odometria refleja lo que de verdad
        paso, en vez de acumular ese error indefinidamente.
        """

        # integrar ruedas (solo para referencia/joint_states opcional)
        self.alpha_L += alphaL_dot_real * dt
        self.alpha_R += alphaR_dot_real * dt

        # velocidad yaw REAL del chasis, medida (no comandada)
        omega_c = (self.R / self.d2) * (alphaR_dot_real - alphaL_dot_real)
        self.theta_c += omega_c * dt

        # velocidad global REAL del punto de control P, via cinematica
        # directa (misma matriz M, pero multiplicando en vez de resolver)
        vx_real, vy_real, theta_p_dot_real = self.forward_point_velocity(
            alphaL_dot_real, alphaR_dot_real, phi_dot_real)

        # integrar plataforma absoluta
        self.theta_p += theta_p_dot_real * dt

        # integrar posicion del robot con la velocidad REAL, no la deseada
        self.x += vx_real * dt
        self.y += vy_real * dt

        return omega_c, theta_p_dot_real, vx_real, vy_real

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
    def publish_odometry(self, omega_c, vx_real, vy_real):
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
        # child_frame_id (local/base_link). Ahora usamos la velocidad
        # REAL (medida), convertida de vuelta a frame local -- ya no la
        # velocidad comandada.
        vx_real_local, vy_real_local = self.global_to_local(
            vx_real, vy_real, self.theta_c)
        od.twist.twist.linear.x = vx_real_local
        od.twist.twist.linear.y = vy_real_local
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

        # 1. Calcular: velocidades de rueda COMANDADAS (lo que le pedimos
        #    al robot que haga)
        alphaL_dot_cmd, alphaR_dot_cmd, phi_dot_cmd = self.compute_kinematics()

        # 2. Integrar: usando las velocidades REALES medidas en
        #    /joint_states (self.alphaL_dot_real, etc.), NO las comandadas.
        #    Esto es lo que corrige la deriva de la odometria.
        omega_c, theta_p_dot_real, vx_real, vy_real = self.integrate_state(
            self.alphaL_dot_real, self.alphaR_dot_real, self.phi_dot_real, dt)

        # 3. Publicar Comandos (estos si son los deseados/comandados)
        self.publish_commands(alphaL_dot_cmd, alphaR_dot_cmd, phi_dot_cmd)

        # 4. Publicar Joint States (SOLO si no hay joint_state_broadcaster
        #    real publicando estos mismos joints -> ver comentario en __init__)
        if self.publish_model_joint_states:
            self.publish_joint_states(alphaL_dot_cmd, alphaR_dot_cmd, phi_dot_cmd, omega_c)

        # 5. Publicar Odometria (con la velocidad REAL, no la comandada)
        self.publish_odometry(omega_c, vx_real, vy_real)

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