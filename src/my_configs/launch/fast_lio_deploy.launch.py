import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to the OFFICIAL launch files
    my_configs_dir = get_package_share_directory('my_configs')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # 2. Path to ALL your custom config files
    fast_lio_config = os.path.join(my_configs_dir, 'config', 'mid360.yaml')
    livox_driver_config = os.path.join(my_configs_dir, 'config', 'MID360_config.json')
    ublox_gps_config = os.path.join(my_configs_dir, 'config', 'zed_f9p.yaml')
    ntrip_client_config = os.path.join(my_configs_dir, 'config', 'ntrip_client.yaml')
   
    # 3. Include the Livox Driver Launch with its config
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_configs_dir, 'launch', 'msg_MID360_launch.py')
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

    # Path to Keithley Launch
    keithley_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('keithley_dmm'), 'launch', 'keithley_yaml_launch.py')
        )
    )

    # Then add 'keithley_launch' to your LaunchDescription return list


    # 5. GPS Node (U-Blox)
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='screen',
        parameters=[ublox_gps_config]
    )

    # 6. NTRIP Client Node (Corrections)
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_client_node',
        output='screen',
        parameters=[ntrip_client_config]
    )

    return LaunchDescription([
        livox_launch,
        fast_lio_launch,
        ublox_node,
        ntrip_node,
        keithley_launch,
    ])