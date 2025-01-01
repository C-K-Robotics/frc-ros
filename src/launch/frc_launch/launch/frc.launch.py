from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from environs import Env

env = Env()
env.read_env("stack.env")

launch_dir = get_package_share_directory("frc_launch")
nt_bridge_dir = get_package_share_directory("networktable_bridge")

launch_ntb = DeclareLaunchArgument(
    "launch_ntb",
    default_value=str(env.bool("LAUNCH_NETWORKTABLE")),
    description="If we should launch networktable bridge for subscribing all NT topics",
)

ntb_node = Node(
    package="networktable_bridge",
    executable="nt_client_sub_node",
    name="ntb_sub_all",
    output="screen",
    parameters=[
        {"NT_server_ip": env.str("ROBOT_IP")},
        {"sampling_time": 0.1},
        {"automated": True},
    ],
    condition=IfCondition(LaunchConfiguration("launch_ntb")),
)


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare arguments
            launch_ntb,
            
            # Include components conditionally
            ntb_node,
            # rde_launch,
            # rpp_launch,
        ]
    )
