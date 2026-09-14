"""Runtime defaults shared by direct estimator execution and launch files."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def configure_image_transport():
    """Use image-sized Fast DDS SHM buffers unless the operator supplied a profile.

    Must run before rclpy.init(). The profile retains UDP discovery/network
    transport and does not change any topic's reliability or history depth.
    """
    for variable in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        if os.environ.get(variable):
            return os.environ[variable]
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp') not in (
            'rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return None
    profile = Path(get_package_share_directory('estimation_pkg')) / 'config/fastdds_images.xml'
    if not profile.is_file():
        raise FileNotFoundError(f'{profile} is missing; rebuild estimation_pkg.')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    return str(profile)
