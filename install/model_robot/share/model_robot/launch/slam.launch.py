import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("model_robot"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',  # Sincroniza con el encabezado del YAML
        output='screen',
        parameters=[
            slam_params_file, # Carga las reglas del archivo de configuración
            {
                'use_sim_time': True,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'transform_timeout': 1.0
            } # Sobreescritura de seguridad directa en el diccionario del nodo
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld