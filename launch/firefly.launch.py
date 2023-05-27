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
                '/world/swarm_world/model/turtle1/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle2/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle3/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle4/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle5/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle3/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle4/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle5/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle1/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle2/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle3/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle4/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle5/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/turtle1/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle2/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle3/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle4/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle5/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',  
                '/model/turtle1/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle2/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle3/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle4/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle5/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',  
                
                '/model/turtle6/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle7/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle8/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle9/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/model/turtle10/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/world/swarm_world/model/turtle6/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle7/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle8/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle9/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle10/link/lidar/sensor/hls_lfcd_lds/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/world/swarm_world/model/turtle6/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle7/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle8/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle9/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle10/link/camera_link/sensor/wide_angle_camera1/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle6/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle7/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle8/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle9/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/world/swarm_world/model/turtle10/link/camera_link/sensor/wide_angle_camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/model/turtle6/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle7/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle8/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle9/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',
                '/model/turtle10/LED_mode@std_msgs/msg/Empty@gz.msgs.Empty',     
                '/model/turtle6/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle7/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle8/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle9/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',
                '/model/turtle10/LED_color@std_msgs/msg/ColorRGBA@gz.msgs.Color',                 
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
    ])
