from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, AppendEnvironmentVariable,
                             IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

DEFAULT_WORLD = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_world.world')
LAUNCH_DIR = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
ROS_GZ_SIM = get_package_share_directory('ros_gz_sim')


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=DEFAULT_WORLD,
        description='Path to Gazebo world file'
    )
    world = LaunchConfiguration('world')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_SIM, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_DIR, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(LAUNCH_DIR, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': '-2.0',
            'y_pose': '-0.5'
        }.items()
    )

    gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    return LaunchDescription([
        world_arg,
        gz_resource,
        gz_server,
        gz_client,
        robot_state_pub,
        spawn_robot,
    ])
