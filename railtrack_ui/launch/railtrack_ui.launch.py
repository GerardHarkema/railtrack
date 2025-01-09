from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the path to the package's share directory
    package_share_directory = get_package_share_directory('railtrack_ui')
    #config_file = "track_config.json"
    #locomotive_images_path = "config/locomotive_images"
    #railtracklayout_images_path = "config/railtracklayout_images"

    # Declare a launch argument to pass the package directory
    return LaunchDescription([
        DeclareLaunchArgument(
            'package_directory',
            default_value=package_share_directory,
            description='Path to the railtrack_ui package directory'
        ),
        
        Node(
            package='railtrack_ui',
            executable='railtrack_ui.py',
            output='screen',
            parameters=[{'package_directory': LaunchConfiguration('package_directory')},
    #                    {'config_file': config_file},
    #                    {"locomotive_images_path": locomotive_images_path},
    #                    {"railtracklayout_images_path": railtracklayout_images_path}
           ],
        ),
    ])
