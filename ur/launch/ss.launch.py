#!/usr/bin/env python3

#SPAWNER CON GAZEBO 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():

    description_package = "ur"

    robots = []

    ur_type_left         = 'ur5e'
    tf_prefix_left       = 'ur_left_'
    xyz_left             = '"-0.46 0 0"'
    rpy_left             = '"0 -1.57 0"'
    finger_tip_cor_left  = '"0.157 0 0.0018"'
    robot_ip_left        = "192.168.0.102"

    hardware_interface   = 'hardware_interface/PositionJointInterface'
    


    robots.append({'ur_type_left': ur_type_left, 'tf_prefix_left': tf_prefix_left, 'parent_left': tf_prefix_left + 'tool0', 'xyz_left': xyz_left,
                   'rpy_left': rpy_left, 'finger_tip_cor_left': finger_tip_cor_left,
                   'hardware_interface': hardware_interface, 'robot_ip_left':robot_ip_left, })

 
    spawn_robots_cmds = IncludeLaunchDescription(
            (os.path.join(get_package_share_directory(description_package), 'launch',
                                                    'one_arm.launch.py')),
            launch_arguments={
                            'ur_type_left': ur_type_left,
                            'tf_prefix_left': tf_prefix_left,
                            'parent_left': tf_prefix_left + 'tool0',
                            'xyz_left': xyz_left,
                            'rpy_left': rpy_left,
                            'finger_tip_cor_left': finger_tip_cor_left,
                            'hardware_interface': hardware_interface,
                            'robot_ip_left': robot_ip_left,
                            }.items())

    # Activate controllers
    def activate_controllers(name, active=True):

        in_active_flags = ["inactive"] if not active else ["active"]
        return ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state',in_active_flags, name],
            output='screen'
        )
    
    controller_spawner_names = [
            "joint_state_broadcaster", 
        ]
    controller_spawner_inactive_names = [ ]

    controller_spawners = [activate_controllers(name) for name in controller_spawner_names] + [
        activate_controllers(name, active=False) for name in controller_spawner_inactive_names
    ]

    # Create the launch description and populate
    ld = LaunchDescription()

    
    ld.add_action(spawn_robots_cmds)

    for controller_spawner in controller_spawners:
        ld.add_action(controller_spawner)

    return ld