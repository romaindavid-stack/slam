import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Paths to the OFFICIAL launch files
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # 2. Paths to YOUR TWO custom config files
    # This one is for FAST-LIO (Mapping/Math)
    fast_lio_config = os.path.join(
        get_package_share_directory('my_configs'), 'config', 'mid360.yaml')

    # This one is for the Driver (Hardware/IP/Connection)
    livox_driver_config = os.path.join(
        get_package_share_directory('my_configs'), 'config', 'MID360_config.json')

    # 3. Include the Livox Driver Launch with its config
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        ),
        # Pass the driver config here
        launch_arguments={'user_config_path': livox_driver_config}.items()
    )

    # 4. Include the Fast-LIO Mapping Launch with its config
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': fast_lio_config}.items()
    )

    return LaunchDescription([
        livox_launch,
        fast_lio_launch
    ])