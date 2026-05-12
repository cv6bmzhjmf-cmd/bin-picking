"""Launch Gazebo bin picking simulation with stereo camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    world_file = LaunchConfiguration('world_file', default=os.path.join(
        pkg_dir, 'worlds', 'bin_picking.world'))

    declare_world = DeclareLaunchArgument(
        'world_file', default_value=world_file,
        description='Path to Gazebo world file')

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        gazebo,
    ])
