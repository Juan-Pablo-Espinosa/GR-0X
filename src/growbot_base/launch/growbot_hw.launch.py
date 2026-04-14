# growbot_hw.launch.py — real hardware only.
# No namespace arg; growbot_both.launch.py wraps this in a PushRosNamespace
# when it needs to run the real side under /real/.
import os
import sys

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import controller_yaml, load_robot_description  # noqa: E402


def generate_launch_description():
    robot_description = load_robot_description(use_sim=False, ns='')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controller_yaml()],
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    return LaunchDescription([
        rsp,
        cm,
        RegisterEventHandler(OnProcessStart(target_action=cm, on_start=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
    ])
