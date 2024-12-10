from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from environs import Env

env = Env()
env.read_env("stack.env")

launch_dir = get_package_share_directory("tools_launch")
gc_dir = get_package_share_directory("ghost_car")

launch_gc = DeclareLaunchArgument(
    "launch_gc",
    default_value=str(env.bool("LAUNCH_GHOST_CAR")),
    description="If we should use MCC in place of RVC",
)

gc_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gc_dir, "launch", "ghost_car.launch.py")),
    launch_arguments={
        "ttl_dir": TextSubstitution(
            text=os.path.join(
                get_package_share_directory("race_metadata"), "ttls", env.str("TTL_FOLDER")
            )
        ),
        "vehicle_name": TextSubstitution(text=env.str("VEHICLE_NAME")),
        "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
        "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
        "controller_type": TextSubstitution(text=env.str("GHOST_CAR_CONTROLLER")),
    }.items(),
    condition=IfCondition(LaunchConfiguration("launch_gc")),
)


def generate_launch_description():
    return LaunchDescription(
        [launch_gc, gc_launch]
    )
