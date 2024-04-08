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
            default_value="dual_env.urdf.xacro",
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
            default_value="0.75",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw_val",
            default_value="1.57",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_roll_val",
            default_value="0.523599",
        ))

    # Initialize Arguments Left
    ur_type_left        = LaunchConfiguration("ur_type_left")
    tf_prefix_left      = LaunchConfiguration("tf_prefix_left")
    xyz_left            = LaunchConfiguration('xyz_left')
    parent_left         = LaunchConfiguration("parent_left")
    rpy_left            = LaunchConfiguration('rpy_left')
    finger_tip_cor_left = LaunchConfiguration('finger_tip_cor_left')
    robot_ip_left       = LaunchConfiguration("robot_ip_left")

    # Initialize Arguments Right
    ur_type_right        = LaunchConfiguration("ur_type_right")
    tf_prefix_right      = LaunchConfiguration("tf_prefix_right")
    xyz_right            = LaunchConfiguration('xyz_right')
    parent_right         = LaunchConfiguration("parent_right")
    rpy_right            = LaunchConfiguration('rpy_right')
    finger_tip_cor_right = LaunchConfiguration('finger_tip_cor_right')
    robot_ip_right       = LaunchConfiguration("robot_ip_right")

    safety_limits       = LaunchConfiguration("safety_limits")
    safety_pos_margin   = LaunchConfiguration("safety_pos_margin")
    safety_k_position   = LaunchConfiguration("safety_k_position")

    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file    = LaunchConfiguration("description_file")
    
    #Camera Arguments 
    spawn_x_val         = LaunchConfiguration("spawn_x_val")
    spawn_y_val         = LaunchConfiguration("spawn_y_val")
    spawn_z_val         = LaunchConfiguration("spawn_z_val")
    spawn_yaw_val       = LaunchConfiguration("spawn_yaw_val")
    spawn_roll_val       = LaunchConfiguration("spawn_roll_val")

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
                 (os.path.join(get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py')),
                 launch_arguments={'physics': 'false'}.items())
    
    # Spawn from topic                                                      
    gazebo_spawn_robot_description = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-topic', 'robot_description',
                        '-entity', 'dual_robot',
                        '-timeout', '5',
                        '-unpause'
                        
                     ],)

    sdf_path_camera = PathJoinSubstitution(
        [FindPackageShare("model"), "objects/2d_camera", "camera_model.sdf"]
    )

    sdf_path_cable = PathJoinSubstitution(
        [FindPackageShare("dlo_model_gazebo"), "worlds", "dlo.sdf"]
    )
    # Spawn from model
    gazebo_spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'distorted_camera',
                   '-file', sdf_path_camera,
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val,
                    '-P', spawn_roll_val,
                    ])
    
    gazebo_spawn_world = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'cable',
                   '-file', sdf_path_cable,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',
                    ])
    

    # MoveIt! configuration
    moveit_config = (
        MoveItConfigsBuilder("dual",package_name="dual_moveit")
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

    # Servo node for realtime control
    servo_yaml_left = load_yaml("ur", "config/ur_servo_left.yaml") 
    servo_yaml_right = load_yaml("ur", "config/ur_servo_right.yaml")
    servo_params_left = {"moveit_servo": servo_yaml_left}
    servo_params_right = {"moveit_servo": servo_yaml_right}
    servo_node_left = Node(
        package="moveit_servo",
        # condition=IfCondition(launch_servo),
        executable="servo_node_main",
        namespace="ur_left",
        parameters=[
            servo_params_left,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            launch_ros.parameter_descriptions.ParameterFile(
                    param_file=ros2_control),
            moveit_config.planning_pipelines,
            moveit_config.planning_scene_monitor,
        ],
        output="screen",
    )

    servo_node_right = Node(
        package="moveit_servo",
        # condition=IfCondition(launch_servo),
        executable="servo_node_main",
        namespace="ur_right",
        parameters=[
            servo_params_right,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            launch_ros.parameter_descriptions.ParameterFile(
                    param_file=ros2_control),
            moveit_config.planning_pipelines,
            moveit_config.planning_scene_monitor,
        ],
        output="screen",
    )

    # Publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[spawn_x_val, spawn_y_val, spawn_z_val, "1.57", spawn_roll_val, "0.0", "world", "camera_link"],
    )

    static_tf_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "-1.57", "0.0", "-1.57", "camera_link", "camera_link_optical"],
    )
    
    cable_init = Node(
        package="dlo_model_gazebo",
        executable="pub_model.py",
        output="screen",
        arguments=['-package_path', 'dlo_model_gazebo', 
                   '-name_dlo_config_file', 'dlo_config.yaml'],
    )
    nodes_to_start = [

        robot_state_publisher_node,    
        gazebo,
        gazebo_spawn_robot_description,
        gazebo_spawn_camera,
        gazebo_spawn_world,
        move_group_node,
        servo_node_left,
        static_tf,
        static_tf_2,
        
        #servo_node_right,
 
    ]

    return LaunchDescription(
        [RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn_world,
                on_exit=cable_init,
            )
        ) ]+
        declared_arguments +
        nodes_to_start) 
    
