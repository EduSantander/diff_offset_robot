from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('mob_rob_arm_description'), 'urdf', 'mob_rob_arm.urdf.xacro'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' arm_yaw:=', LaunchConfiguration('arm_yaw')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('arm_yaw', default_value='0.0'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])