import os
import math
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    mycobot_description_share_path = os.path.join(get_package_share_directory('gazebo_simulation_ros2'))
    world_path = PathJoinSubstitution([FindPackageShare("gazebo_simulation_ros2"), "world", "my_world.world"])

    use_hardware = LaunchConfiguration('use_hardware')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_use_hardware_cmd = DeclareLaunchArgument(
        name='use_hardware',
        default_value='false',
        description='use myCobot hardware'
    )
    
    # Robot description  
    robot_xacro_file = os.path.join(mycobot_description_share_path, 'urdf', 'mycobot_280_m5_gazebo_config.xacro')
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_roll_val = '0.00'
    spawn_pitch_val = '0.00'
    cal_spawn_yaw_val = -(math.pi) / 2
    spawn_yaw_val = str(cal_spawn_yaw_val)

    gazebo_share_directory = get_package_share_directory('gazebo_simulation_ros2')
    gazebo_ros_share_directory = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_directory, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(gazebo_share_directory, 'world', 'my_world.world')}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_directory, 'launch', 'gzclient.launch.py'))
    )
    
    doc = xacro.parse(open(robot_xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    gazebo_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mycobot_ai_no",
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-R', spawn_roll_val,
                   '-P', spawn_pitch_val,
                   '-Y', spawn_yaw_val],
        parameters=[
            {'use_sim_time': True},
        ],
        output="screen",
    )
    
    # Static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )
     
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    # MoveIt2 config   
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # Planning context
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("mycobot_moveit_config", "config/mycobot_ai_no.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }
    
    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
       
    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    } 
    ompl_planning_yaml = load_yaml("mycobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("mycobot_moveit_config", "config/mycobot_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True}, 
        ],
    )
    
    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("mycobot_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True}, 
        ],
        condition=UnlessCondition(load_RVIZfile),
    )
    
    # mycobot hardware
    mycobot_hardware_interface_node = Node(
        package='gazebo_simulation_ros2',
        executable='slider_control.py',
        condition=IfCondition(use_hardware),
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ]
    )

    ld = LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_use_hardware_cmd,
        gzserver,
        gzclient,
        gazebo_spawn_entity,
        static_tf,
        robot_state_publisher,        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = load_joint_trajectory_controller,
                on_exit = [
                    # MoveIt!2:
                    TimerAction(
                        period=1.0,
                        actions=[
                            rviz_arg,
                            rviz_node_full,
                            run_move_group_node,
                            mycobot_hardware_interface_node
                        ]
                    ),
                ]
            )
        )
    ])
    return ld