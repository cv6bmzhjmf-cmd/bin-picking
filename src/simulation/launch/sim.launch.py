"""Launch Gazebo bin picking simulation with stereo camera + UR5."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default=os.path.join(
        get_package_share_directory('bin_picking_sim'), 'worlds', 'bin_picking.world'))

    declare_world = DeclareLaunchArgument(
        'world_file', default_value=world_file,
        description='Path to Gazebo world file')

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Script paths: launch/ → up to share/pkg/ → src/vision/
    _pkg_share = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    _vision_dir = os.path.join(_pkg_share, 'src', 'vision')

    stereo_matcher = ExecuteProcess(
        cmd=['python3', os.path.join(_vision_dir, 'stereo_matcher.py')],
        output='screen'
    )

    pose_estimator = ExecuteProcess(
        cmd=['python3', os.path.join(_vision_dir, 'pose_estimator.py')],
        output='screen'
    )

    grasp_planner = ExecuteProcess(
        cmd=['python3', os.path.join(_vision_dir, 'grasp_planner.py')],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        stereo_matcher,
        pose_estimator,
        grasp_planner,
    ])
