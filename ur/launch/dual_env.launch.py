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

    ur_type_right        = 'ur5e'
    tf_prefix_right      = 'ur_right_'
    xyz_right            = '"0.46 0 0"'
    rpy_right            = '"0 -1.57 0"'
    finger_tip_cor_right = '"0.157 0 0.0018"'
    robot_ip_right       = "192.168.0.103"

    hardware_interface   = 'hardware_interface/PositionJointInterface'
    


    robots.append({'ur_type_left': ur_type_left, 'tf_prefix_left': tf_prefix_left, 'parent_left': tf_prefix_left + 'tool0', 'xyz_left': xyz_left,
                   'rpy_left': rpy_left, 'finger_tip_cor_left': finger_tip_cor_left,
                   'hardware_interface': hardware_interface, 'robot_ip_left':robot_ip_left,
                   'ur_type_right': ur_type_right, 'tf_prefix_right': tf_prefix_right,'parent_right': tf_prefix_right + 'tool0', 
                   'xyz_right': xyz_right,'rpy_right': rpy_right, 'finger_tip_cor_right': finger_tip_cor_right,"robot_ip_right": robot_ip_right})

    spawn_robots_cmds = []

    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory(description_package), 'launch',
                                                           'gazebo.launch.py')),
                launch_arguments={
                                    'ur_type_left': robot['ur_type_left'],
                                    'tf_prefix_left': robot['tf_prefix_left'],
                                    'parent_left': robot['parent_left'],
                                    'xyz_left': robot['xyz_left'],
                                    'rpy_left': robot['rpy_left'],
                                    'finger_tip_cor_left': robot['finger_tip_cor_left'],
                                    'hardware_interface': robot['hardware_interface'],
                                    'robot_ip_left': robot['robot_ip_left'],
                                    'ur_type_right': robot['ur_type_right'],
                                    'tf_prefix_right': robot['tf_prefix_right'],
                                    'parent_right': robot['parent_right'],
                                    'xyz_right': robot['xyz_right'],
                                    'rpy_right': robot['rpy_right'],
                                    'finger_tip_cor_right': robot['finger_tip_cor_right'],
                                    'robot_ip_right': robot['robot_ip_right'],
                                  }.items()))
    
    # Activate controllers
    def activate_controllers(name, active=True):

        in_active_flags = ["inactive"] if not active else ["active"]
        return ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state',in_active_flags, name],
            output='screen'
        )
    
    controller_spawner_names = [
            "joint_state_broadcaster", 
            "ur_left_joint_group_vel_controller",
            "ur_right_joint_group_vel_controller",
            #"ur_left_hand_controller"
            #"cable_joint_group_vel_controller"
            #"ur_left_joint_group_pos_controller",
        ]
    controller_spawner_inactive_names = [ ]

    controller_spawners = [activate_controllers(name) for name in controller_spawner_names] + [
        activate_controllers(name, active=False) for name in controller_spawner_inactive_names
    ]

    # Create the launch description and populate
    ld = LaunchDescription()

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for controller_spawner in controller_spawners:
        ld.add_action(controller_spawner)

    # robot_chain = Node(
    #     package="ur",
    #     executable="pub_model.py",
    #     output="screen",
    #     arguments=['-package_path', 'dlo_model_gazebo', 
    #                '-name_dlo_config_file', 'dlo_config.yaml'],
    # )

    return ld