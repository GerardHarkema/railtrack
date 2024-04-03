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


    config_file = "/home/gerard/railtrack_ws/src/railtrack/railtrack_ui/../config/track_config_DCC.json"
    locomotive_images_path = "/home/gerard/railtrack_ws/src/railtrack/railtrack_ui/../config/locomotive_images"
    railtracklayout_images_path = "/home/gerard/railtrack_ws/src/railtrack/railtrack_ui/../config/railtracklayout_images"

    return LaunchDescription([
        Node(
            package='railtrack_ui',
            executable='railtrack_ui.py',
            output='screen',
            parameters=[{'config_file': config_file},
                        {"locomotive_images_path": locomotive_images_path},
                        {"railtracklayout_images_path": railtracklayout_images_path}
                        ],
        ),
    ])
