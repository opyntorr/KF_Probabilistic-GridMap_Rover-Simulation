import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tello_gazebo_dir = get_package_share_directory('tello_gazebo')
    ros_ign_gazebo_dir = get_package_share_directory('ros_ign_gazebo')

    world_sdf = os.path.join(tello_gazebo_dir, 'worlds', 'simple.sdf')
    tello_sdf = os.path.join(tello_gazebo_dir, 'models', 'tello', 'model.sdf')
    bridge_config = os.path.join(tello_gazebo_dir, 'config', 'bridge.yaml')

    return LaunchDescription([
        # 1. Launch Gazebo Sim (Ignition)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_ign_gazebo_dir, 'launch', 'ign_gazebo.launch.py')
            ),
            launch_arguments={'ign_args': f'-r {world_sdf}'}.items()
        ),

        # 2. Spawn Tello
        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'drone1',
                '-file', tello_sdf,
                '-x', '0', '-y', '0', '-z', '1'
            ],
            output='screen'
        ),

        # 3. ROS Ign Bridge (CLI args: [ = IGN->ROS, ] = ROS->IGN)
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/drone1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/drone1/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/drone1/camera_down@sensor_msgs/msg/Image[ignition.msgs.Image',
            ],
            output='screen'
        ),

        # 4. Robot State Publisher (Optional but good for RViz)
        # Since we are using SDF, we might need a URDF for state publisher if needed,
        # but for now let's stick to the basics.
    ])
