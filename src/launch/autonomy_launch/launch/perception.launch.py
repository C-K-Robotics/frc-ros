"""Generates the launch description for the entire perception stack.
"""

import os
from environs import Env
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# Load environment variables
env = Env()
env.read_env("stack.env")

# Create launch argument to determine if we should launch perception
launch_perception = DeclareLaunchArgument(
    "launch_perception",
    default_value=str(env.bool("LAUNCH_PERCEPTION")),
    description="If perception stack should be launched",
)

# ----------------------------------------
# Base Perception Stack
# ----------------------------------------
# Get ROS2 package share directories

# Create launch descriptions for each module in the base perception stack

# ----------------------------------------
# Classic Perception Stack
# ----------------------------------------
# Get ROS2 package share directories
# autoware_perception_launch_dir = get_package_share_directory("autoware_perception_launch")
# polygon_transformer_dir = get_package_share_directory("polygon_transformer")

# Create launch descriptions for each module in the classic perception stack

# autoware_perception_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(autoware_perception_launch_dir, "launch", "autoware_perception.launch.py")
#     ),
#     launch_arguments={
#         "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
#     condition=IfCondition(LaunchConfiguration("launch_perception")),
# )

# polygon_transformer_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(polygon_transformer_dir, "launch", "polygon_transformer.launch.py")
#     ),
#     launch_arguments={
#         "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
#         "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
#     }.items(),
#     condition=IfCondition(LaunchConfiguration("launch_perception")),
# )

# ----------------------------------------
# Modern Perception Stack
# ----------------------------------------
# Get ROS2 package share directories
yolov8_dir = get_package_share_directory("yolov8")
lidarcam_proj_v8_dir = get_package_share_directory("lidarcam_proj_v8")
multi_car_tracker_dir = get_package_share_directory("multi_car_tracker")

# Create launch descriptions for each module in the modern perception stack

yolov8_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(yolov8_dir, "launch", "yolov8.launch.py")),
    launch_arguments={}.items(),  # No arguments
    condition=IfCondition(LaunchConfiguration("launch_perception")),
)

lidarcam_proj_v8_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(lidarcam_proj_v8_dir, "launch", "l2c_v8.launch.py")
    ),
    launch_arguments={}.items(),  # No arguments
    condition=IfCondition(LaunchConfiguration("launch_perception")),
)

multi_car_tracker_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(multi_car_tracker_dir, "launch", "backup_clustering.launch")
    ),
    launch_arguments={}.items(),  # No arguments
    condition=IfCondition(LaunchConfiguration("launch_perception")),
)


def generate_launch_description():
    return LaunchDescription(
        [
            launch_perception,
            # autoware_perception_launch,
            # polygon_transformer_launch,
            yolov8_launch,
            lidarcam_proj_v8_launch,
            multi_car_tracker_launch,
        ]
    )
