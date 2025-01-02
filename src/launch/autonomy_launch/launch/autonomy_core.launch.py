# Copyright 2024 C.K. Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from base_common import get_share_file
import os
from environs import Env

env = Env()
env.read_env("stack.env")

launch_dir = get_package_share_directory("autonomy_launch")
foxglove_dir = get_package_share_directory("foxglove_bridge")
# rde_dir = get_package_share_directory("race_decision_engine")
# rpp_dir = get_package_share_directory("race_path_planner")

launch_foxglove = DeclareLaunchArgument(
    "launch_foxglove",
    default_value=str(env.bool("LAUNCH_FOXGLOVE")),
    description="If we should use foxglove_bridge",
)

foxglove_launch = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        os.path.join(foxglove_dir, "launch", "foxglove_bridge_launch.xml")
    ),
    condition=IfCondition(LaunchConfiguration("launch_foxglove")),
)

# rde_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(rde_dir, "launch", "race_decision_engine.launch.py")
#     ),
#     launch_arguments={
#         "map_dir": TextSubstitution(
#             text=get_share_file("common_metadata", "maps", env.str("MAP_FOLDER"))
#         ),
#         "game_type": TextSubstitution(text=env.str("GAME_TYPE")),
#         "robot_name": TextSubstitution(text=env.str("ROBOT_NAME")),
#     }.items(),
# )

# rpp_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(os.path.join(rpp_dir, "launch", "race_path_planner.launch.py")),
#     launch_arguments={
#         "map_dir": TextSubstitution(
#             text=get_share_file("common_metadata", "maps", env.str("MAP_FOLDER"))
#         ),
#         "game_type": TextSubstitution(text=env.str("GAME_TYPE")),
#         "robot_name": TextSubstitution(text=env.str("ROBOT_NAME")),
#     }.items(),
# )


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare arguments
            launch_foxglove,
            
            # Include components conditionally
            foxglove_launch,
            # rde_launch,
            # rpp_launch,
        ]
    )
