# IF THE ORIGINAL ROBOT_LAUNCH SOMEHOW GETS DELETED, MOVE THIS INTO THE install/six_axis_pubsub/share/six_axis_pubsub FOLDER AND REMOVE THE _back IN THE FILENAME
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare the path to the controller configuration file
    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=os.path.join(
            os.getenv('HOME'),
            'Documents/Code/learnRos/src/six_axis_pubsub/src/controller_config.yaml'
        ),
        description='Path to the controller configuration file'
    )

    # Path to the URDF file
    urdf_file_path = os.path.join(
        os.getenv('HOME'),
        'Documents/Code/learnRos/src/six_axis_pubsub/src/robot_arm.urdf'
    )

    return LaunchDescription([
        # Declare the controller configuration argument
        controller_config_arg,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        # Node for the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

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
