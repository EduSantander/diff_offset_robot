import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('model_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Rutas
    xacro_file = os.path.join(pkg_share, 'urdf', 'diff_offset_robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_config.rviz')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'room_with_pillars.world')

    # Procesar XACRO
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    # Fix conocido de gazebo_ros2_control: quitar comentarios XML
    robot_desc = re.sub(r'<!--.*?-->', '', robot_desc, flags=re.DOTALL)

    # Gazebo, ahora cargando NUESTRO mundo en vez del vacio por default
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

    # slam_toolbox en modo mapeo (por ahora, sin argumentos de localizacion)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
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
        gazebo,
        rsp,
        spawn,
        controllers_after_spawn,
        kinematics_node,
        slam_toolbox_node,
        rviz2
    ])