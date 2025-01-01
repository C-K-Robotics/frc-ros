import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    launch_dir = get_package_share_directory("sensor_launch")
    name = LaunchConfiguration('name').perform(context)
    rgb_topic_name = name+'/rgb/image_raw'
    if LaunchConfiguration('rectify_rgb').perform(context)=='true':
        rgb_topic_name = name +'/rgb/image_rect'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'launch', 'oakd_camera_base.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file,
                              "parent_frame": LaunchConfiguration("parent_frame"),
                               "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                               "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                               "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                               "cam_roll": LaunchConfiguration("cam_roll"),
                               "cam_pitch": LaunchConfiguration("cam_pitch"),
                               "cam_yaw": LaunchConfiguration("cam_yaw"),
                               "use_rviz": LaunchConfiguration("use_rviz")
                               }.items()),

        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("rectify_rgb")),
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="rectify_color_node",
                        remappings=[('image', name+'/rgb/image_raw'),
                                    ('camera_info', name+'/rgb/camera_info'),
                                    ('image_rect', name+'/rgb/image_rect'),
                                    ('image_rect/compressed', name+'/rgb/image_rect/compressed'),
                                    ('image_rect/compressedDepth', name+'/rgb/image_rect/compressedDepth'),
                                    ('image_rect/theora', name+'/rgb/image_rect/theora')]
                    )
            ]),
        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('depth_registered/image_rect', name+'/stereo/image_raw'),
                                ('rgb/image_rect_color', rgb_topic_name),
                                ('rgb/camera_info', name+'/rgb/camera_info'),
                                ('points', name+'/points')]
                    ),
            ],
        ),
        # LoadComposableNodes(
        #     target_container=name+"_container",
        #     composable_node_descriptions=[
        #             ComposableNode(
        #                 package="depthai_filters",
        #                 plugin="depthai_filters::Detection2DOverlay",
        #                 remappings=[('rgb/preview/image_raw', name+'/nn/passthrough/image_raw'),
        #                             ('nn/detections', name+'/nn/detections')]
        #             ),
        #     ],
        # ),
        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::SpatialBB",
                        name="spatial_bb_node",
                        remappings=[
                                    ('stereo/camera_info', name+'/stereo/camera_info'),
                                    ('nn/spatial_detections', name+'/nn/spatial_detections'),
                                    ('rgb/preview/image_raw', name+'/rgb/preview/image_raw'),
                                    ],
                        parameters=[params_file],
                    ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    launch_dir = get_package_share_directory("sensor_launch")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="camera_link"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(launch_dir, 'config', 'oakd_yolo.param.yaml')),
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument("rectify_rgb", default_value="True"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
