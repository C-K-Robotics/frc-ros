from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import json
from base_common import get_share_file, get_sim_time_launch_arg, to_lower, check_val_in_list
from environs import Env

# Read the mapping config file
uuids = json.load(open("install/frc_launch/share/frc_launch/param/uuids.json", "r"))

env = Env()
env.read_env("stack.env")

launch_dir = get_package_share_directory("frc_launch")
nt_bridge_dir = get_package_share_directory("networktable_bridge")
sensor_dir = get_package_share_directory("sensor_launch")

launch_ntb = DeclareLaunchArgument(
    "launch_ntb",
    default_value=str(env.bool("LAUNCH_NETWORKTABLE")),
    description="If we should launch networktable bridge for subscribing all NT topics",
)

map_dir_arg = DeclareLaunchArgument(
    "map_dir", default_value="None", description="MAP Directory to use"
)

robot_name_arg = DeclareLaunchArgument(
    "robot_name",
    default_value=str(env.str("ROBOT_NAME")),
    description="Which Robot to use",
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

frc_config = (
    get_share_file("frc_launch"),
    "/param/",
    to_lower(LaunchConfiguration("robot_name")),
    "/frc.param.yaml",
)

SENSORS = env.list("SENSORS")
sensors_launch = []
for SENSOR in SENSORS:
    # get sensor UUID
    sensor_uuid = uuids["SENSORS"].get(SENSOR, None)
    if sensor_uuid is None:
        raise ValueError(
            f"Sensor {SENSORS} not found in mapping config file.\n"
            + f"Available sensors: {list(uuids['SENSORS'].keys())}"
        )

    if "OAKD" in SENSOR:
        if "PCL" in SENSOR:
            oakd_launch_file = "oakd_pcl.launch.py"
        elif "YOLO" in SENSOR:
            oakd_launch_file = "oakd_yolo.launch.py"
        else:
            oakd_launch_file = "oakd.launch.py"
        sensors_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensor_dir, "launch", oakd_launch_file)
                ),
                launch_arguments={
                    "name": sensor_uuid,
                    "parent_frame": sensor_uuid + "_frame",
                    "params_file": frc_config,
                }.items(),
            )
        )
    elif "MID360" in SENSOR:
        sensors_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensor_dir, "launch", "lidar_MID360.launch.py")
                ),
            )
        )
    elif "LD06" in SENSOR:
        sensors_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensor_dir, "launch", "lidar_ld06.launch.py")
                ),
            )
        )
    else:
        raise ValueError(
            f"Sensor {SENSOR} is not supported.\n"
            + f"Available sensors: {list(uuids['SENSORS'].keys())}"
        )


def generate_launch_description():

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(launch_ntb)
    ld.add_action(map_dir_arg)
    ld.add_action(robot_name_arg)

    # Include components conditionally
    ld.add_action(ntb_node)

    # Include sensors
    for launch in sensors_launch:
        ld.add_action(launch)
    
    return ld
