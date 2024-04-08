# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
import launch_ros.parameter_descriptions
from ur_moveit_config.launch_common import load_yaml
from ament_index_python import get_package_prefix

from launch import LaunchDescription
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type_left",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type_right",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_gripper.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_left",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_right",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
            "xyz_left",
            default_value=TextSubstitution(text="0 0 0"),
            )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
            "xyz_right",
            default_value=TextSubstitution(text="0 0 0"),
            )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parent_left",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "parent_right",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        "rpy_left",
        default_value=TextSubstitution(text="0 0 0"),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        "rpy_right",
        default_value=TextSubstitution(text="0 0 0"),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        "finger_tip_cor_left",
        default_value=TextSubstitution(text="0 0 0"),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
        "finger_tip_cor_right",
        default_value=TextSubstitution(text="0 0 0"),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_left", 
            default_value="192.168.0.102",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_right", 
            default_value="192.168.0.103",
            description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="true",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. \
            The user has be be allowed to write to this location. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "generate_ros2_control_tag", default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )

    # Initialize Arguments
    ur_type_left        = LaunchConfiguration("ur_type_left")
    tf_prefix_left      = LaunchConfiguration("tf_prefix_left")
    xyz_left            = LaunchConfiguration('xyz_left')
    parent_left         = LaunchConfiguration("parent_left")
    rpy_left            = LaunchConfiguration('rpy_left')
    finger_tip_cor_left = LaunchConfiguration('finger_tip_cor_left')

    ur_type_right        = LaunchConfiguration("ur_type_right")
    tf_prefix_right      = LaunchConfiguration("tf_prefix_right")
    xyz_right            = LaunchConfiguration('xyz_right')
    parent_right         = LaunchConfiguration("parent_right")
    rpy_right            = LaunchConfiguration('rpy_right')
    finger_tip_cor_right = LaunchConfiguration('finger_tip_cor_right')

    safety_limits     = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    robot_ip_left = LaunchConfiguration("robot_ip_left")
    robot_ip_right = LaunchConfiguration("robot_ip_right")

    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    generate_ros2_control_tag = LaunchConfiguration("generate_ros2_control_tag")
    launch_servo = LaunchConfiguration("launch_servo")


    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )

    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "xacro", description_file]),
            " ",
            "robot_ip_left:=",
            robot_ip_left,
            " ",
            "robot_ip_right:=",
            robot_ip_right,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type_left:=",
            ur_type_left,
            " ",
            "tf_prefix:=",
            tf_prefix_left,
            " ",
            "xyz_left:=",
            xyz_left,
            " ",
            "parent_left:=",
            parent_left,
            " ",
            "rpy_left:=",
            rpy_left,
            " ",
            "finger_tip_cor_left:=",
            finger_tip_cor_left,
            " ",
            "ur_type_right:=",
            ur_type_right,
            " ",
            "tf_prefix_right:=",
            tf_prefix_right,
            " ",
            "xyz_right:=",
            xyz_right,
            " ",
            "parent_right:=",
            parent_right,
            " ",
            "rpy_right:=",
            rpy_right,
            " ",
            "finger_tip_cor_right:=",
            finger_tip_cor_right,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "generate_ros2_control_tag:=",
            generate_ros2_control_tag,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
    MoveItConfigsBuilder("dual",package_name="moveit_config")
    .robot_description(file_path="config/dual.urdf.xacro")
    .robot_description_semantic(file_path="config/dual.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()])
    
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rviz_config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )

    # Static TF

    tf_left = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur_left_base_link"]
    tf_right = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur_right_base_link"]
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur_right_base_link"],
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # ros2_control real world
    real_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur"), "config", "arm_controller.yaml"]
    )
    real_gripper_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("robotiq_hand_e"), "config", "robotiq_controller.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description,ros2_controllers_path],
        remappings=[('joint_states', 'joint_state_broadcaster')],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[robot_description, launch_ros.parameter_descriptions.ParameterFile(
                    param_file = real_joint_controllers,
                    allow_substs=True)],
        remappings=[('joint_states', 'joint_state_broadcaster')],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    #Spawn controllers
    def controller_spawner(name, active=True):
        inactive_flags = ["--inactive"] if not active else []
        print("spawning controller: ", name)
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags,
            
        )

    if not(use_fake_hardware):
        controller_spawner_names = [
            "joint_state_broadcaster",
            "ur_right_joint_group_pos_controller","ur_left_joint_group_pos_controller","ur_right_joint_group_vel_controller",
        ]
    else:
        #sim
        controller_spawner_names = [
           #"joint_state_broadcaster",
            # "ur_right_joint_group_vel_controller",
            "ur_right_arm_controller", 
            "ur_right_hand_controller","ur_left_hand_controller",
            "ur_left_arm_controller",
           
            
        ]
# 
# "ur_right_arm_controller",

    controller_spawner_inactive_names = [ "ur_right_joint_group_pos_controller","ur_left_joint_group_pos_controller"]

    
    controller_spawners = [controller_spawner(name) for name in controller_spawner_names] + [
        controller_spawner(name, active=False) for name in controller_spawner_inactive_names
    ]

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        )


    # Servo node for realtime control
    servo_yaml = load_yaml("ur", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # launch_ros.parameter_descriptions.ParameterFile(
            #         param_file=real_joint_controllers)
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.planning_scene_monitor,
        ],
        output="screen",
    )

    gazebo_spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', 'robot_description',
                        '-entity', 'dual_robot',
                        '-timeout', '5'],)

    

    nodes_to_start = [
        
        #static_tf,
        robot_state_publisher_node,
        move_group_node,
        ros2_control_node,
        ur_control_node,
        joint_state_publisher,
        # servo_node,
        gazebo_spawn,
        rviz_node,
 
    ]

    return LaunchDescription(declared_arguments + nodes_to_start + controller_spawners ) 
    
