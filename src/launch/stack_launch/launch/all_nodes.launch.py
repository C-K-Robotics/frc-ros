import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml

launch_file_path = os.path.split(os.path.realpath(__file__))[0] + '/'
node_list_input_path = os.path.join(launch_file_path, '../param/node_config.yaml') # '/home/jetson/projects/robocar/src/launch/basestation_launch/param/node_config.yaml'
node_packages_info_path = os.path.join(launch_file_path, '../param/node_pkg_locations.yaml') # '/home/jetson/projects/robocar/src/launch/basestation_launch/param/node_pkg_locations.yaml'

def update_parameters(node_list_input_path):
        with open(node_list_input_path, "r") as file:
            car_inputs = yaml.load(file, Loader=yaml.FullLoader)
            my_car_inputs = {}
            for key in car_inputs:
                value = car_inputs[key]
                if value==1:
                    my_car_inputs[key] = value
            return my_car_inputs


def update_packages(node_packages_info_path):
    with open(node_packages_info_path, "r") as file:
            packages_dict = yaml.load(file, Loader=yaml.FullLoader)
            car_inputs_dict = update_parameters(node_list_input_path)
            my_packages = {}
            for key in packages_dict:
                value = packages_dict[key]
                if key in car_inputs_dict:
                    my_packages[key] = value
            return my_packages


def generate_a_launch_description(some_package, some_launch):
    return LaunchDescription([
    DeclareLaunchArgument(
            'topic_name',
            default_value = 'default_value',
            description = 'REQUIRED argument for new topic name (can use original topic name if needed)'
        ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(some_package),
                    'launch',
                    some_launch)
            ),
        ),
    ])


def generate_launch_description():
    my_packages_dict = update_packages(node_packages_info_path)
    ld = LaunchDescription()
    for key in my_packages_dict:
        pkg_name = my_packages_dict[key][0]
        launch_name = my_packages_dict[key][1]
        try:
            ld.add_action(generate_a_launch_description(pkg_name, launch_name))
        except:
            pass
        print(f"Trying to start: {launch_name} from {pkg_name}")
    return ld
