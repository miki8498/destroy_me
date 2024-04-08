#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    description_package = "ur"

    robots = []

    ur_type_left         = 'ur5e'
    ur_type_right        = 'ur5e'
    tf_prefix_left       = 'ur_left_'
    tf_prefix_right      = 'ur_right_'
    xyz_left             = '"-0.46 0 0"'
    xyz_right            = '"0.46 0 0"'
    rpy_left             = '"0 -1.57 0"'
    finger_tip_cor_left  = '"0.157 0 0.0018"'
    rpy_right            = '"0 -1.57 0"'
    finger_tip_cor_right = '"0.157 0 0.0018"'
    hardware_interface   = 'hardware_interface/PositionJointInterface'
    robot_ip_left        = "192.168.0.102"
    robot_ip_right       = "192.168.0.103"
    use_fake_hardware    = "false"
    fake_sensor_commands = "false"
    generate_ros2_control_tag = "true"

    robots.append({'ur_type_left': ur_type_left, 'tf_prefix_left': tf_prefix_left, 'parent_left': tf_prefix_left + 'tool0', 'xyz_left': xyz_left,
                   'rpy_left': rpy_left, 'finger_tip_cor_left': finger_tip_cor_left,
                   'ur_type_right': ur_type_right, 'tf_prefix_right': tf_prefix_right,'parent_right': tf_prefix_right + 'tool0', 'xyz_right': xyz_right, 
                   'rpy_right': rpy_right, 'finger_tip_cor_right': finger_tip_cor_right,
                   'hardware_interface': hardware_interface, 'robot_ip_left':robot_ip_left, "robot_ip_right": robot_ip_right, "use_fake_hardware": use_fake_hardware,
                   "fake_sensor_commands": fake_sensor_commands, "generate_ros2_control_tag": generate_ros2_control_tag})

    spawn_robots_cmds = []

    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory(description_package), 'launch',
                                                           'spawn_robot.launch.py')),
                launch_arguments={
                                  'ur_type_left': robot['ur_type_left'],
                                  'tf_prefix_left': robot['tf_prefix_left'],
                                  'parent_left': robot['parent_left'],
                                  'xyz_left': robot['xyz_left'],
                                  'rpy_left': robot['rpy_left'],
                                  'finger_tip_cor_left': robot['finger_tip_cor_left'],
                                  'ur_type_right': robot['ur_type_right'],
                                  'tf_prefix_right': robot['tf_prefix_right'],
                                  'parent_right': robot['parent_right'],
                                  'xyz_right': robot['xyz_right'],
                                  'rpy_right': robot['rpy_right'],
                                  'finger_tip_cor_right': robot['finger_tip_cor_right'],
                                  'hardware_interface': robot['hardware_interface'],
                                  'robot_ip_right': robot['robot_ip_right'],
                                  'robot_ip_left': robot['robot_ip_left'],
                                  'use_fake_hardware': robot['use_fake_hardware'],
                                  'fake_sensor_commands': robot['fake_sensor_commands'],
                                  'generate_ros2_control_tag': robot['generate_ros2_control_tag'],
                                  'launch_servo': 'true',
                                  }.items()))

    gazebo_process = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen')

    ld = LaunchDescription()
    
  
    #ld.add_action(gazebo_process)

    for spawn_robot_cmd in spawn_robots_cmds:
    
        ld.add_action(spawn_robot_cmd)

    return ld