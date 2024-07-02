import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    pkg_name = 'nt_client'
    some_node = 'nt_client_node'    
    some_config = 'nt_client.yaml'

    config = os.path.join(
        get_package_share_directory(pkg_name),
        'param',
        some_config)
        
    client_node=Node(
        executable=some_node,
        package=pkg_name,
        output='screen',
        parameters=[config],
    )

    ld = LaunchDescription()
    ld.add_action(client_node)
    
    return ld
