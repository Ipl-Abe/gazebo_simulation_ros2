import os
import math
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

from launch_ros.actions import Node


import xacro


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
        default_value='true',
        description='use myCobot hardware'
    )
        

    robot_xacro_file = os.path.join(mycobot_description_share_path, 'urdf', 'mycobot_280_m5_gazebo_config.xacro')

    # Gazebo Node
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
    params = {'robot_description': doc.toxml()}

    gazebo_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mycobot",
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
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            params,
            {'use_sim_time': True},
        ]
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[
            {'use_sim_time': True},
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
    
    mycobot_hardware_interface_node = Node(
        package='gazebo_simulation_ros2',
        executable='slider_control.py',
        condition=IfCondition(use_hardware),
        output='screen',
        parameters=[
            params,
            {'use_sim_time': True},
        ]
    )
    
    rqt_joint_trajectory_controller_node = Node(
        package='rqt_joint_trajectory_controller',
        executable='rqt_joint_trajectory_controller',
        condition=IfCondition(use_hardware),
        output='screen',
        parameters=[
            params,
            {'use_sim_time': True},
        ]
    )
    
    

    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["--x", spawn_x_val,
                   "--y", spawn_y_val,
                   "--z", spawn_z_val,
                   "--roll", spawn_roll_val,
                   "--pitch", spawn_pitch_val,
                   "--yaw", spawn_yaw_val,
                   "--frame-id", "world",
                   "--child-frame-id", "link1"],
        parameters=[
            {"use_sim_time": True},
        ]
    )
    
    ld = LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_use_hardware_cmd,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gazebo_spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        # declare_use_sim_time_cmd,
        # declare_world_cmd,
        gzserver,
        gzclient,
        gazebo_spawn_entity,
        robot_state_publisher_node,
        # mycobot_hardware_interface_node,
        # rqt_joint_trajectory_controller_node
    ])

    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_world_cmd)
    # ld.add_action(gzserver)
    # ld.add_action(gzclient)
    # ld.add_action(gazebo_spawn_entity)
    # ld.add_action(robot_state_publisher_node)
 
 
    # ld.add_action(joint_state_broadcaster_node)
    # ld.add_action(joint_trajectory_controller_node)
    # ld.add_action(static_transform)

    return ld
