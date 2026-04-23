import os 
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    spawn_positions = {
        'easy': {'x': '0.0', 'y': '-3.5', 'z': '0.01', 'yaw': '1.57'},
        'medium': {'x': '0.28', 'y': '-7.8', 'z': '0.01', 'yaw': '1.57'},
        'hard': {'x': '0.0', 'y': '-8', 'z': '0.01', 'yaw': '1.57'}
    }
    
    difficulty_value = 'easy'
    for arg in sys.argv:
        if arg.startswith('difficulty:='):
            difficulty_value = arg.split(':=')[1]
            break
    
    spawn_pos = spawn_positions.get(difficulty_value, spawn_positions['easy'])
    
    difficulty_arg = DeclareLaunchArgument(
        'difficulty',
        default_value='easy',
        description='Difficulty level: easy, medium, or hard'
    )
    
    difficulty = LaunchConfiguration('difficulty')

    filename = [difficulty, TextSubstitution(text='.sdf')]

    package_share_dir = get_package_share_directory("robot_simulation")
    mapping_share_dir = get_package_share_directory("mapping_pkg")
    control_share_dir = get_package_share_directory("control_pkg")

    world_path = PathJoinSubstitution([
        package_share_dir,
        'worlds',
        filename  
    ])
    

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    launch_path = os.path.join(
            get_package_share_directory("robot_simulation"),
            "launch"
    )

    gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")
            ),
            launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    gz_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")    
            ),
            launch_arguments = {"gz_args": ["-g -v1"]}.items()
    )
    

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"robot_state_publisher.launch.py")
             )
    )

    spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"spawn_entity.launch.py")
            ),
            launch_arguments={
                'x': spawn_pos['x'],
                'y': spawn_pos['y'],
                'z': spawn_pos['z'],
                'yaw': spawn_pos['yaw']
            }.items()
    )

    lidar_processor = Node(
            package = "perception_pkg",
            executable="lidar_processor_node",
    )

    mapping_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mapping_share_dir,"launch","map_visulator.launch.py")
            ),
            launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    wall_follower_node = Node(
        package = "control_pkg",
        executable="wall_follower_node",
        name='wall_follower_node',
        output='screen',
        parameters = [{
                "use_sim_time":True,
        }]
    )


    return LaunchDescription([
        difficulty_arg,
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_entity,
        lidar_processor,
        mapping_launch,
        wall_follower_node
    ])
