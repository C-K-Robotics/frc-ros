import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

import yaml

def yaml_decode(yaml_input_path):
        with open(yaml_input_path, "r") as file:
            yaml_inputs = yaml.load(file, Loader=yaml.FullLoader)
            numeric_inputs = {}
            string_inputs = {}
            for key in yaml_inputs:
                value = yaml_inputs[key]
                if isinstance(value, float) or isinstance(value, int):
                    numeric_inputs[key] = value
                elif isinstance(value, str):
                    string_inputs[key] = value
            return numeric_inputs, string_inputs

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

launch_dir = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = launch_dir + '../config/livox_config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

slice_config_path = os.path.join(cur_config_path, 'slice_frame_info.yaml')
pcl_to_scan_config = os.path.join(cur_config_path, 'pcl_to_scan.yaml')

slice_info_num, slice_info_str = yaml_decode(slice_config_path)
# print(slice_info_num, '\n', slice_info_str)

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    
    slice_frame_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[str(slice_info_num['x']),
                    str(slice_info_num['y']),
                    str(slice_info_num['z']),
                    str(slice_info_num['yaw']),
                    str(slice_info_num['pitch']),
                    str(slice_info_num['roll']),
                    slice_info_str['frame_id'],
                    slice_info_str['child_frame_id']]
    )

    pcl_to_scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', slice_info_str['pointcloud_topic']),
                    # ('scan', [LaunchConfiguration(variable_name='scanner'), slice_info_str['slice_scan_topic']]),
                    ('scan', slice_info_str['slice_scan_topic'])],
        parameters=[pcl_to_scan_config],
        name='pointcloud_to_laserscan'
    )

    return LaunchDescription(
        [
            livox_driver,
            slice_frame_tf_pub,
            pcl_to_scan,
        ]
    )
