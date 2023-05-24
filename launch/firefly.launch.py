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
                '/model/turtle1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle5/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle6/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle7/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle8/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle9/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle10/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle3/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle4/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle5/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle6/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle7/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle8/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle9/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle10/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle3/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle4/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle5/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle6/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle7/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle8/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle9/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle10/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/turtle11/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle12/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle13/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle14/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle15/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle16/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle17/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle18/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle19/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle20/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                """ '/model/turtle11/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle12/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle13/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle14/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle15/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle16/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle17/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle18/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle19/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle20/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/world/swarm_world/model/turtle11/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle12/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle13/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle14/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle15/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle16/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle17/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle18/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle19/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle20/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle11/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle12/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle13/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle14/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle15/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle16/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle17/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle18/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle19/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle20/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/turtle11/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle12/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle13/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle14/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle15/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle16/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle17/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle18/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle19/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle20/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty', """
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
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle3"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle4"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle5"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle6"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle7"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle8"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle9"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle10"}
            ]
        ),
        """ Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle11"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle12"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle13"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle14"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle15"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle6"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle17"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle18"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle19"}
            ]
        ),
        Node(
            package='firefly',
            executable='firefly',
            parameters=[
                {"model_name": "turtle20"}
            ]
        ), """
    ])
