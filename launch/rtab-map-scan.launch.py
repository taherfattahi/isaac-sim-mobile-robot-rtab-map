from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the rtabmap_demos package
    rtabmap_demos_dir = get_package_share_directory('rtabmap_demos')
    turtlebot3_scan_launch_path = os.path.join(rtabmap_demos_dir, 'launch', 'turtlebot3_scan.launch.py')

    # Include the turtlebot3_scan.launch.py from rtabmap_demos
    turtlebot3_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_scan_launch_path)
    )

    # Return the LaunchDescription
    return LaunchDescription([turtlebot3_scan_launch])
