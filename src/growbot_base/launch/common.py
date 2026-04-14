"""Shared helpers for the growbot launch helps keep things in sync
   
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory

PKG = 'growbot_base'


def pkg_share() -> str:
    return get_package_share_directory(PKG)


def xacro_path() -> str:
    return os.path.join(pkg_share(), 'urdf', 'growbot.urdf.xacro')


def controller_yaml() -> str:
    return os.path.join(pkg_share(), 'config', 'controller.yaml')


def bridge_yaml() -> str:
    return os.path.join(pkg_share(), 'config', 'bridge.yaml')


def load_robot_description(use_sim: bool, ns: str = '') -> str:
    """Process growbot.urdf.xacro into a URDF string.
    basically we have the gazebo and the hardware, by using different flags we can activate one or both
    also good for future usage(I hope)
    """
    return xacro.process_file(
        xacro_path(),
        mappings={
            'use_sim': 'true' if use_sim else 'false',
            'ns': ns,
        },
    ).toxml()
