from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # dit moet nog anders referentio t.o.v. package
    # wijzig niet de python file, maar de template


    railtrack_ui_path = "/home/gerard/railtrack_ws/src/railtrack/railtrack_ui"

    return LaunchDescription([
        Node(
            package='railtrack_ui',
            executable='railtrack_emulator.py',
            output='screen',
            parameters=[{"railtrack_ui_path": railtrack_ui_path}
                        ],
        ),
    ])
