import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('model_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    map_file = LaunchConfiguration('map_file')

    # Rutas
    xacro_file = os.path.join(pkg_share, 'urdf', 'diff_offset_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_config.rviz')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'room_with_pillars.world')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Procesar XACRO
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    # Fix conocido de gazebo_ros2_control: quitar comentarios XML
    robot_desc = re.sub(r'<!--.*?-->', '', robot_desc, flags=re.DOTALL)

    # Gazebo, cargando el mundo del cuarto con pilares
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode', default_value='localization',
        description='localization (usar tu mapa ya guardado, default) o mapping (mapear desde cero)'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file', default_value='/home/edu/diff_offset_robot/mi_mapa',
        description='Ruta SIN extension al pose-graph (mi_mapa.posegraph + mi_mapa.data), '
                    'solo se usa si slam_mode=localization'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diff_offset_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.05'
        ],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_wheels = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller'],
        output='screen'
    )

    load_platform = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['platform_controller'],
        output='screen'
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[load_jsb, load_wheels, load_platform]
        )
    )

    kinematics_node = Node(
        package='kinematics_controller',
        executable='kinematics_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'R': 0.055},
            {'d1': 0.11225},
            {'d2': 0.36},
            {'publish_model_joint_states': False},
        ],
        output='screen'
    )

    # NUEVO: control de orientacion de la plataforma, independiente
    # del chasis. Ver platform_orientation_controller.py
    platform_orientation_node = Node(
        package='kinematics_controller',
        executable='platform_orientation_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'Kp_theta_p': 1.5},
            {'max_phi_dot': 3.0},
        ],
        output='screen'
    )

    is_localization = PythonExpression(["'", slam_mode, "' == 'localization'"])
    is_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])

    slam_toolbox_mapping_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time, 'mode': 'mapping'}
        ],
        condition=IfCondition(is_mapping)
    )

    slam_toolbox_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'mode': 'localization',
                'map_file_name': map_file,
                'map_start_pose': [0.0, 0.0, 0.0],
            }
        ],
        condition=IfCondition(is_localization)
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_mode_cmd,
        declare_map_file_cmd,
        gazebo,
        rsp,
        spawn,
        controllers_after_spawn,
        kinematics_node,
        platform_orientation_node,
        slam_toolbox_mapping_node,
        slam_toolbox_localization_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        rviz2
    ])