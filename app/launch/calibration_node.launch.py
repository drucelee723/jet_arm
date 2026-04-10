import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

def launch_setup(context):
    compiled = os.environ['need_compile']
    
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    enable_base = LaunchConfiguration('base').perform(context)

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')
        ),
        condition=IfCondition(enable_base),  
    )

    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')
        ),
        condition=IfCondition(enable_base),
    )

    calibration_node = Node(
        package='app',
        executable='calibration',
        output='screen',
    )

    return [
        depth_camera_launch,
        sdk_launch,
        calibration_node,
    ]

def generate_launch_description():
    base_arg = DeclareLaunchArgument(
        'base',
        default_value='true',  
    )

    return LaunchDescription([
        base_arg,
        OpaqueFunction(function=launch_setup),
    ])

if __name__ == '__main__':
    from launch import LaunchService
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
