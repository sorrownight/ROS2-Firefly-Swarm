import os
import sys, signal, subprocess, time


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
                'gz', 'sim', '-r',
                os.path.join(
                    pkg_firefly,
                    'worlds',
                    'swarm_world.sdf'
                )
            ],
            shell=True
        ),
        # Launch a bridge to forward ROS2's Twist msg to Gazebo's Twist under /cmd_vel topic
        # We have to do this for Gazebo Garden
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            
            arguments=[ # Gazebo made me do this. Held as a mental hostage doing this.
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/turtle1/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle2/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',            
            ],
        ),
        # Launch firefly nodes
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle1"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle2"}
            ]
        ),
    ])
