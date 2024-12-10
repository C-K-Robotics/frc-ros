from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from base_common import get_share_file
import os
from environs import Env

env = Env()
env.read_env("stack.env")

launch_dir = get_package_share_directory("basestation_launch")
rt_dir = get_package_share_directory("race_telemetry")

launch_joystick = DeclareLaunchArgument(
    "launch_joystick",
    default_value=str(env.bool("LAUNCH_JOYSTICK")),
    description="If we should use joystick",
)

vis_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(rt_dir, "launch", "visualization.launch.py")),
    launch_arguments={
        "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
        "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
    }.items(),
)


def generate_launch_description():
    return LaunchDescription(
        [
            launch_joystick,
            vis_launch,
        ]
    )
