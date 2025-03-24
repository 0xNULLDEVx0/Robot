from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

def generate_robot_state_publisher_node(context, *args, **kwargs):
    # Get the URDF file path from the launch argument
    urdf_file_path = LaunchConfiguration('urdf_file').perform(context)

    # Read the content of the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Return the robot_state_publisher node with the robot_description parameter
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ]

def generate_launch_description():
    # Declare the controller configuration file as a launch argument
    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=os.path.join(
            os.getenv('HOME'),
            'Documents/Code/learnRos/src/six_axis_pubsub/src/controller_config.yaml'
        ),
        description='Path to the controller configuration file'
    )

    # Declare the URDF file path as a launch argument
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(
            os.getenv('HOME'),
            'Documents/Code/learnRos/src/six_axis_pubsub/src/robot_arm.urdf'
        ),
        description='Path to the URDF file'
    )

    return LaunchDescription([
        # Declare the controller configuration argument
        controller_config_arg,
        # Declare the URDF file argument
        urdf_file_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Node for the robot state publisher
        OpaqueFunction(function=generate_robot_state_publisher_node),

        # Node for the ROS2 control manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[LaunchConfiguration('controller_config')],
            output='screen'
        ),

        # Node for the joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Node for RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'config.rviz']
        )
    ])
    