import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml
import launch_ros.parameter_descriptions
from launch.event_handlers import OnProcessExit

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
            default_value="single_env.urdf.xacro",
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
            "xyz_left",
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
        "rpy_left",
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
            "robot_ip_left", 
            default_value="192.168.0.102",
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x_val",
            default_value="0",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y_val",
            default_value="-0.5",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z_val",
            default_value="0.6",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw_val",
            default_value="1.57",
        ))

    # Initialize Arguments Left
    ur_type_left        = LaunchConfiguration("ur_type_left")
    tf_prefix_left      = LaunchConfiguration("tf_prefix_left")
    xyz_left            = LaunchConfiguration('xyz_left')
    parent_left         = LaunchConfiguration("parent_left")
    rpy_left            = LaunchConfiguration('rpy_left')
    finger_tip_cor_left = LaunchConfiguration('finger_tip_cor_left')
    robot_ip_left       = LaunchConfiguration("robot_ip_left")

    safety_limits       = LaunchConfiguration("safety_limits")
    safety_pos_margin   = LaunchConfiguration("safety_pos_margin")
    safety_k_position   = LaunchConfiguration("safety_k_position")

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file    = LaunchConfiguration("description_file")
    

    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )

    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    ros2_control = PathJoinSubstitution(
        [FindPackageShare("ur"), "config", "arm_controller.yaml"]
    )
    
    #Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "xacro", description_file]),
            " ",
            "robot_ip_left:=",
            robot_ip_left,
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
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "simulation_controllers:=",
            ros2_control,
            " ",
    
        ]
    )
    robot_description = {"robot_description": robot_description_content}

   
    #Open gazebo and spawn robots
    gazebo = IncludeLaunchDescription(
                 (os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py')))
    print(gazebo)
    
    # Spawn from topic                                                      
    gazebo_spawn_robot_description = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', 'robot_description',
                        '-entity', 'single_robot',
                        '-timeout', '5',
                        '-unpause'
                        
                     ],)


    # Publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    nodes_to_start = [

        robot_state_publisher_node,    
        gazebo,
        gazebo_spawn_robot_description,
 
    ]

    return LaunchDescription(
        declared_arguments +
        nodes_to_start) 
    
