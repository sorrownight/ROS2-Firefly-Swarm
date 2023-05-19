import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_firefly = get_package_share_directory('firefly')
    return LaunchDescription([
        # Launch gazebo (includes listener)
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r', '-v 4',
                os.path.join(
                    pkg_firefly,
                    'worlds',
                    'swarm_world.sdf'
                )
            ]
        ),
        # Launch a bridge to forward ROS2's Twist msg to Gazebo's Twist under /cmd_vel topic
        # We have to do this for Gazebo Garden
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
        ),
        # Launch firefly publisher
        Node(
            package='firefly',
            executable='firefly',
        )
    ])
