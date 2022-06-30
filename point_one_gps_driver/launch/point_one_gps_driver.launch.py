from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    bringup_dir = get_package_share_directory('point_one_gps_driver')

    param_file_path = os.path.join(
        bringup_dir,
        'param',
        'point_one_gps_driver.param.yaml'
    )
    
    gps_node = Node(
        package='point_one_gps_driver',
        executable='point_one_gps_driver_node_exe',
        output='screen',
        parameters=[
            param_file_path
        ],
        remappings={
            'nav_sat_fix':'/gps/fix',
            'gps_fix': '/gps/gps',
            'imu': '/gps/imu',
            'pose': '/gps/pose'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(gps_node)

    return ld