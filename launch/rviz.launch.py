from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')

    # Include the rviz_launch.py from nav2_bringup
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )

    # Return the LaunchDescription
    return LaunchDescription([rviz_launch])
