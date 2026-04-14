
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

SIM_NS = 'sim'
REAL_NS = 'real'
JTC_TOPIC = 'joint_trajectory_controller/joint_trajectory'


def generate_launch_description():
    pkg_share = get_package_share_directory('growbot_base')
    sim_launch = os.path.join(pkg_share, 'launch', 'growbot_sim.launch.py')
    hw_launch = os.path.join(pkg_share, 'launch', 'growbot_hw.launch.py')

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'ns': SIM_NS}.items(),
    )

    include_hw = GroupAction([
        PushRosNamespace(REAL_NS),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(hw_launch)),
    ])

    relay_to_sim = Node(
        package='topic_tools',
        executable='relay',
        name='mirror_relay_sim',
        arguments=[f'/{JTC_TOPIC}', f'/{SIM_NS}/{JTC_TOPIC}'],
        output='screen',
    )

    relay_to_real = Node(
        package='topic_tools',
        executable='relay',
        name='mirror_relay_real',
        arguments=[f'/{JTC_TOPIC}', f'/{REAL_NS}/{JTC_TOPIC}'],
        output='screen',
    )

    return LaunchDescription([
        include_sim,
        include_hw,
        relay_to_sim,
        relay_to_real,
    ])
