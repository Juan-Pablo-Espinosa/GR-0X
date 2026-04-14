#launches SIM only
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

#use common.py
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from common import bridge_yaml, load_robot_description  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('ns').perform(context)
    bridge_name = LaunchConfiguration('bridge_name').perform(context)

    robot_description = load_robot_description(use_sim=True, ns=ns)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )


    spawn_growbot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_growbot',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-topic', 'robot_description',
            '-name', f'growbot{("_" + ns) if ns else ""}',
            '-x', '0', '-y', '0', '-z', '1.5',
        ],
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=bridge_name,
        output='screen',
        parameters=[{'config_file': bridge_yaml()}],
    )

    #polls gazebo until its up then excecutes the chain
    gz_ready = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until IGN_PARTITION=growbot ign service --list 2>/dev/null '
            '| grep -q "/world/empty/control"; '
            'do sleep 0.5; done; echo "Gazebo ready"'
        ],
        output='screen',
    )
    #the chain, processes MUST by done sequentially or else they will launch simultaniously and break
    after_gz = RegisterEventHandler(
        OnProcessExit(target_action=gz_ready, on_exit=[spawn_growbot])
    )
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_growbot, on_exit=[load_jsb])
    )
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_controller])
    )

    group_children = [
        robot_state_publisher,
        parameter_bridge,
        gz_ready,
        after_gz,
        after_spawn,
        after_jsb,
    ]
    #push all ros actions to namespace
    if ns:
        return [GroupAction([PushRosNamespace(ns)] + group_children)]
    return group_children


def generate_launch_description():
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='',
                              description='ROS namespace for controller_manager + topics.'),
        DeclareLaunchArgument('bridge_name', default_value='growbot_bridge'),
        SetEnvironmentVariable(name='IGN_PARTITION', value='growbot'),
        gz_sim,
        OpaqueFunction(function=_launch_setup),
    ])
