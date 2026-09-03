#!/usr/bin/env python3
"""
Evaluador automatico de precision de Nav2 para el robot diferencial con
offset (omnidireccionalidad virtual).

Manda una matriz de goals sistematicos (distancia x angulo de giro final),
mide el error REAL de posicion/orientacion via TF (map->base_link) al
terminar cada intento, y guarda todo en un CSV para su analisis posterior.

Requisitos:
    sudo apt install ros-humble-nav2-simple-commander

Uso (con robot_nav2.launch.py YA corriendo en otra terminal):
    python3 nav2_eval.py
"""

import math
import csv
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def quat_to_yaw_deg(qz, qw):
    return math.degrees(2.0 * math.atan2(qz, qw))


def angle_diff_deg(a_deg, b_deg):
    """Diferencia angular mas corta entre dos angulos, resultado en [-180, 180]."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def get_current_pose_map(navigator, tf_buffer):
    try:
        t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw_deg = quat_to_yaw_deg(t.transform.rotation.z, t.transform.rotation.w)
        return x, y, yaw_deg
    except Exception as e:
        navigator.get_logger().warn(f'No pude leer TF map->base_link: {e}')
        return None


def go_and_wait(navigator, pose):
    navigator.goToPose(pose)
    while not navigator.isTaskComplete():
        pass
    return navigator.getResult()


def main():
    rclpy.init()
    navigator = BasicNavigator()

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)  # noqa: F841 (se mantiene vivo)

    navigator._waitForNodeToActivate('bt_navigator')
    
    # ---------------- CONFIGURACION DE LA PRUEBA ----------------
    HOME = (0.0, 0.0, 0.0)          # x, y, yaw_deg -- punto de referencia
    DISTANCIAS = [3.0] #[1.0, 2.0, 3.0]    # metros, siempre a lo largo de +X desde HOME
    ANGULOS = [0.0, 90.0, 180.0]    # grados, orientacion final ABSOLUTA deseada
    REPETICIONES = 3

    # Deben coincidir con nav2_params.yaml (general_goal_checker)
    XY_GOAL_TOLERANCE_M = 0.20 #0.15
    YAW_GOAL_TOLERANCE_DEG = math.degrees(0.2)
    # --------------------------------------------------------------

    results = []
    trial_id = 0
    total = len(DISTANCIAS) * len(ANGULOS) * REPETICIONES

    for dist in DISTANCIAS:
        for ang in ANGULOS:
            for rep in range(1, REPETICIONES + 1):
                trial_id += 1
                print(f'\n=== Prueba {trial_id}/{total}: '
                      f'distancia={dist} m, angulo={ang} deg, rep={rep} ===')

                # 1) Volver a HOME antes de cada prueba real (no se registra)
                print('  Regresando a HOME...')
                go_and_wait(navigator, make_pose(navigator, *HOME))
                time.sleep(1.0)  # dejar que la odometria/TF se asienten

                # 2) Prueba real
                target_x, target_y = dist, 0.0
                goal_pose = make_pose(navigator, target_x, target_y, ang)

                start_time = time.time()
                nav2_result = go_and_wait(navigator, goal_pose)
                elapsed = time.time() - start_time

                result_str = {
                    TaskResult.SUCCEEDED: 'SUCCEEDED',
                    TaskResult.CANCELED: 'CANCELED',
                    TaskResult.FAILED: 'FAILED',
                }.get(nav2_result, 'UNKNOWN')

                time.sleep(0.5)
                final = get_current_pose_map(navigator, tf_buffer)
                if final is None:
                    final_x = final_y = final_yaw = float('nan')
                else:
                    final_x, final_y, final_yaw = final

                pos_error = math.hypot(final_x - target_x, final_y - target_y)
                yaw_error = abs(angle_diff_deg(final_yaw, ang))

                exito = (
                    result_str == 'SUCCEEDED'
                    and pos_error <= XY_GOAL_TOLERANCE_M
                    and yaw_error <= YAW_GOAL_TOLERANCE_DEG
                )

                row = {
                    'trial_id': trial_id,
                    'distancia_objetivo_m': dist,
                    'angulo_objetivo_deg': ang,
                    'repeticion': rep,
                    'nav2_result': result_str,
                    'tiempo_s': round(elapsed, 2),
                    'final_x': round(final_x, 3),
                    'final_y': round(final_y, 3),
                    'final_yaw_deg': round(final_yaw, 2),
                    'error_posicion_m': round(pos_error, 3),
                    'error_orientacion_deg': round(yaw_error, 2),
                    'exito': exito,
                }
                results.append(row)
                print(f'  -> {result_str}, error_pos={pos_error:.3f} m, '
                      f'error_yaw={yaw_error:.2f} deg, exito={exito}')

    # Guardar CSV
    filename = f'nav2_eval_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)
    print(f'\nResultados guardados en: {filename}')

    # Resumen en consola
    exitos = sum(1 for r in results if r['exito'])
    print(f'\nTasa de exito GLOBAL: {exitos}/{total} ({100*exitos/total:.1f}%)')
    print('\nDesglose por angulo de giro:')
    for ang in ANGULOS:
        subset = [r for r in results if r['angulo_objetivo_deg'] == ang]
        e = sum(1 for r in subset if r['exito'])
        print(f'  {ang:>5.0f} grados: {e}/{len(subset)} ({100*e/len(subset):.1f}%)')
    print('\nDesglose por distancia:')
    for dist in DISTANCIAS:
        subset = [r for r in results if r['distancia_objetivo_m'] == dist]
        e = sum(1 for r in subset if r['exito'])
        print(f'  {dist} m: {e}/{len(subset)} ({100*e/len(subset):.1f}%)')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()