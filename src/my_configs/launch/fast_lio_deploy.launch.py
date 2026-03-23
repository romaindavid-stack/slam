import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import time
from launch_ros.actions import Node

def generate_launch_description():
    # 0. Add a toggle so you can choose when to record
    # Usage: ros2 launch my_configs fast_lio_deploy.launch.py record:=true
    record_bag_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Set to "true" to start recording a ROS bag.'
    )
    # Define the recording command
    # We use a timestamp in the filename to avoid overwriting old data
    bag_name = "bags/slam_run_" + time.strftime("%Y_%m_%d-%H_%M_%S")

    bag_recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_name,
            '/ublox_gps_node/fix',
            '/odometry',
            '/livox/lidar',
            '/livox/imu',
            '/keithley/measurement'
        ],
        output='screen'
    )

    # 1. Paths to the OFFICIAL launch files
    my_configs_dir = get_package_share_directory('my_configs')
    fast_lio_dir = get_package_share_directory('fast_lio')

    # 2. Path to ALL your custom config files
    fast_lio_config = os.path.join(my_configs_dir, 'config', 'mid360.yaml')
    livox_driver_config = os.path.join(my_configs_dir, 'config', 'MID360_config.json')
    ublox_gps_config = os.path.join(my_configs_dir, 'config', 'zed_f9p.yaml')
   
    # 3. Include the Livox Driver Launch with its config

    livox_params = {
        "xfer_format": 1,
        "multi_topic": 0,
        "data_src": 0,
        "publish_freq": 10.0,
        "output_data_type": 0,
        "frame_id": 'livox_frame',
        "user_config_path": livox_driver_config,
        "cmdline_input_bd_code": 'livox0000000001'
    }

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[livox_params]
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

    # NTRIP Client (Using its own internal defaults)
    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_configs'), 'launch', 'ntrip_client_launch.py')
        )
    )

    return LaunchDescription([
        record_bag_arg,
        bag_recorder,
        livox_driver_node,
        fast_lio_launch,
        ublox_node,
        ntrip_launch,
        keithley_launch,
    ])