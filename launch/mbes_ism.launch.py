from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("mbes_ism"),
        "config",
        "mbes_ism.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package="mbes_ism",
            executable="mbes_ism_node",
            name="mbes_ism",
            output="screen",
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        )
    ])
