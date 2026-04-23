import os 

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory("mapping_pkg")

    slam_config_file_path = os.path.join(
        package_share_dir,
        'config',
        'slam.yaml'
    )
    
    rviz_config_file_path = os.path.join(
        package_share_dir,
        'config',
        'rviz_config.rviz'
    )


    slam_pkg = get_package_share_directory("slam_toolbox")

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg,"launch","online_async_launch.py")    
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'slam_params_file': slam_config_file_path,
            }.items()
    )

    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file_path] 
    )
    

    return LaunchDescription([
        slam,
        rviz
    ])