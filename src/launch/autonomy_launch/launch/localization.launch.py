import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from environs import Env
from base_common import check_val_in_list


def generate_launch_description():
    env = Env()
    env.read_env("stack.env")
    race_type_arg = DeclareLaunchArgument(
        "race_type",
        default_value=TextSubstitution(text=env.str("RACE_TYPE")),
        description="Race Type",
    )

    velocity_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velocity_estimation"),
                "launch",
                "velocity_estimation.launch.py",
            )
        ),
        launch_arguments={
            "robot_name": TextSubstitution(text=env.str("ROBOT_NAME")),
            "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
        }.items(),
    )

    ekf_bot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_localization_interface"),
                "launch",
                "ekf.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
            "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
            "ekf_param_file": "ekf_bot.param.yaml",
            "ekf_interface_param_file": "ekf_bot_interface.param.yaml",
            "ekf_node_name": "ekf_bot_node",
            "output_odom_topic": "/gps_bot/odometry/filtered",
            "output_accel_topic": "/gps_bot/accel/filtered",
            "output_slip_topic": "/gps_bot/slip",
            "publish_tf": "True",
        }.items(),
        condition=IfCondition(
            check_val_in_list(
                "race_type",
                [
                    "IAC_LVMS",
                    "SVL_LVMS",
                    "AW_SIM_IAC_LVMS",
                ],
            )
        ),
    )

    ekf_top_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_localization_interface"),
                "launch",
                "ekf.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
            "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
            "ekf_param_file": "ekf_top.param.yaml",
            "ekf_node_name": "ekf_top_node",
            "ekf_interface_param_file": "ekf_top_interface.param.yaml",
            "output_odom_topic": "/gps_top/odometry/filtered",
            "output_accel_topic": "/gps_top/accel/filtered",
            "output_slip_topic": "/gps_top/slip",
            "publish_tf": "False",
        }.items(),
        condition=IfCondition(
            check_val_in_list(
                "race_type",
                [
                    "IAC_LVMS",
                    "SVL_LVMS",
                    "AW_SIM_IAC_LVMS",
                ],
            )
        ),
    )

    ekf_hawaii_kart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_localization_interface"),
                "launch",
                "ekf.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": TextSubstitution(text=str(env.bool("USE_SIM_TIME"))),
            "race_type": TextSubstitution(text=env.str("RACE_TYPE")),
            "ekf_param_file": "ekf_bot.param.yaml",
            "ekf_node_name": "ekf_bot_node",
            "ekf_interface_param_file": "ekf_bot_interface.param.yaml",
            "output_odom_topic": "/gps_bot/odometry/filtered",
            "output_accel_topic": "/gps_bot/accel/filtered",
            "output_slip_topic": "/gps_bot/slip",
            "publish_tf": "True",
        }.items(),
        condition=IfCondition(
            check_val_in_list(
                "race_type",
                [
                    "HAWAII_GOKART_AAIS",
                    "HAWAII_GOKART_UHMCP",
                ],
            )
        ),
    )

    return LaunchDescription(
        [
            race_type_arg,
            velocity_estimation_launch,
            ekf_bot_launch,
            ekf_top_launch,
            ekf_hawaii_kart_launch,
        ]
    )
