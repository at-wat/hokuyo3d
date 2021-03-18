"""
<?xml version="1.0"?>
<launch>
  <arg name="basename" default="hokuyo3d"/>

  <node pkg="tf" type="static_transform_publisher" name="stf_$(arg basename)_laser"
      args="0.071 0.0 0.0055 0.0 0.0 0.0 $(arg basename)_link $(arg basename) 100"/>
  <node pkg="tf" type="static_transform_publisher" name="stf_$(arg basename)_imu"
      args="0.01 0.0 0.0 1.571 0.0 1.571 $(arg basename)_link $(arg basename)_imu 100"/>
  <node pkg="tf" type="static_transform_publisher" name="stf_$(arg basename)_mag"
      args="0.01 0.0 0.0 -1.571 -1.571 1.571 $(arg basename)_link $(arg basename)_mag 100"/>

    <node pkg="hokuyo3d" type="hokuyo3d" name="hokuyo3d" output="screen">
        <param name="vertical_interlace" value="10"/> -->
        <param name="horizontal_interlace" value="20"/>
        <param name="vertical_interlace" value="10"/>
        <param name="output_cycle" value="frame"/>
    </node>

</launch>
"""
"""Copyright 2020 Stratom, Inc."""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    argument = 'hokuyo3d'
    config = os.path.join(
        get_package_share_directory('hokuyo3d'),
        'config',
        'hokuyo3d.yaml'
    )

    # Follow node
    hokuyo3d = Node(
        package='hokuyo3d',
        executable='hokuyo3d',
        namespace=argument,
        output='screen',
        parameters=[config],
        # arguments=['--ros-args', '--log-level', "debug"]
    )

    # Publish base link TF
    static_tf_laser = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='screen',
                     arguments=[
                         '0.071',
                         '0.0',
                         '0.055',
                         '0.0',
                         '0.0',
                         '0.0',
                         argument + "_link",
                         argument])

    static_tf_imu = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='screen',
                     arguments=[
                         '0.01',
                         '0.0',
                         '0.0',
                         '1.571',
                         '0.0',
                         '1.571',
                         argument + "_link",
                         argument + "_imu"])

    static_tf_mag = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='screen',
                     arguments=[
                         '0.01',
                         '0.0',
                         '0.0',
                         '-1.571',
                         '-1.571',
                         '1.571',
                         argument + "_link",
                         argument + "_mag"])

    return LaunchDescription([
        static_tf_laser,
        static_tf_imu,
        static_tf_mag,
        hokuyo3d
    ])
