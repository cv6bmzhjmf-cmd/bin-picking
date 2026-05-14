"""Launch Gazebo bin picking simulation with stereo camera + UR5 grasp planning."""
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

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    _pkg_share = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    _vision_dir = os.path.join(_pkg_share, 'src', 'vision')
    _urdf_path = '/tmp/ur5.urdf'

    # Generate UR5 URDF before grasp_planner starts
    _xacro_cmd = (
        'xacro /opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro '
        'name:=ur5 ur_type:=ur5 '
        '| sed "s|package://ur_description|/opt/ros/humble/share/ur_description|g" '
        f'> {_urdf_path}'
    )
    generate_urdf = ExecuteProcess(
        cmd=['bash', '-c', _xacro_cmd],
        output='screen'
    )

    stereo_matcher = ExecuteProcess(
        cmd=['python3', os.path.join(_vision_dir, 'stereo_matcher.py')],
        output='screen'
    )

    pose_estimator = ExecuteProcess(
        cmd=['python3', os.path.join(_vision_dir, 'pose_estimator.py')],
        output='screen'
    )

    # Spawn UR5 in Gazebo (5s delay for Gazebo to fully start)
    spawn_ur5 = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-file', _urdf_path, '-entity', 'ur5',
                     '-x', '0.5', '-y', '0.35', '-z', '0.0'],
                output='screen'
            )
        ]
    )

    # Delay grasp_planner 3s to ensure URDF is generated first
    grasp_planner = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(_vision_dir, 'grasp_planner.py')],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_world,
        gazebo,
        generate_urdf,
        spawn_ur5,
        stereo_matcher,
        pose_estimator,
        grasp_planner,
    ])
