import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_share = get_package_share_directory('model_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'diff_offset_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        output='screen'
    )

    # Spawn robot in Gazebo
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

    # Load controllers **AFTER SPAWN**
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

    # Start controllers only AFTER spawn terminates
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[load_jsb, load_wheels, load_platform]
        )
    )

    # Your kinematics node
    kinematics_node = Node(
        package='kinematics_controller',
        executable='kinematics_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'R': 0.055},
            {'d1': 0.11225},
            {'d2': 0.36}
        ],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
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
        rviz2
    ])
