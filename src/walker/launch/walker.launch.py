from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Walker node launch description
    walker_node = Node(
        package='walker',
        executable='laser_sub',
        name='walker_node',
    )

    # Argument for enabling and disabling rosbag recording
    ros_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable rosbag recording using (true/false)',
    )

    # Adding a logger to get the output if the recording is enabled
    rosbag_log = LogInfo(
        condition = IfCondition(LaunchConfiguration('record_bag')),
        msg='Enabling rosbag recording'
    )


    # Rosbag configuration to record particular topics
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],
        condition = IfCondition(LaunchConfiguration('record_bag')),
        output='screen',
    )

    # Rosbag Timer
    stop_recording = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'ros2 bag record'],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    
    # Launch all the nodes and actions
    return LaunchDescription([
        walker_node,
        ros_bag_arg,
        rosbag_log,
        rosbag,
        stop_recording,
    ])