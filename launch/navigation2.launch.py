from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the 'use_sim_time' argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time if true'
    )
    
    # Path to the navigation_launch.py
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    
    # Include the navigation_launch.py file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        navigation_launch
    ])
