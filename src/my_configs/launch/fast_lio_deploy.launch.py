import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import time
from launch_ros.actions import Node

def generate_launch_description():
    # --- 0. ARGUMENTS & CONFIG ---
    if not os.path.exists('bags'):
        os.makedirs('bags')

    record_bag_arg = DeclareLaunchArgument('record', default_value='false')
    playback_arg = DeclareLaunchArgument('playback', default_value='false')
    # Path to the bag you want to play back (only used if playback:=true)
    bag_file_arg = DeclareLaunchArgument('bag_file', default_value='')

    # Shortcuts for conditions and parameters
    is_playback = LaunchConfiguration('playback')
    is_recording = LaunchConfiguration('record')
    # This ensures nodes use the bag's clock during playback
    use_sim_time_param = {'use_sim_time': is_playback}

    # --- 1. RECORDER & PLAYER ---
    bag_name = "bags/slam_run_" + time.strftime("%Y_%m_%d-%H_%M_%S")
    bag_recorder = ExecuteProcess(
        condition=IfCondition(is_recording),
        cmd=['ros2', 'bag', 'record', '-o', bag_name,
             '/ublox_gps_node/fix', '/odometry', '/livox/lidar',
             '/livox/imu', '/keithley/measurement', '/keithley/geotagged_marker'],
        output='screen'
    )

    bag_player = ExecuteProcess(
        # --clock is essential so nodes see the bag's time
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file'), '--clock'],
        condition=IfCondition(is_playback)
    )

    # --- 2. PATHS ---
    my_configs_dir = get_package_share_directory('my_configs')
    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_config = os.path.join(my_configs_dir, 'config', 'mid360.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_configs'), 'rviz', 'geotagger.rviz')
    geotagger_config = os.path.join(get_package_share_directory('my_configs'), 'config', 'geotagger.yaml')
    # --- 3. HARDWARE NODES (Muted during playback) ---

    livox_driver_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        parameters=[{
            "user_config_path": os.path.join(my_configs_dir, 'config', 'MID360_config.json'),
            "xfer_format": 1, "publish_freq": 10.0, "frame_id": 'livox_frame'
        }],
        condition=UnlessCondition(is_playback),
        output='screen'
    )

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        parameters=[os.path.join(my_configs_dir, 'config', 'zed_f9p.yaml')],
        condition=UnlessCondition(is_playback),
        output='screen'
    )

    keithley_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('keithley_dmm'), 'launch', 'keithley_yaml_launch.py')
        ),
        condition=UnlessCondition(is_playback)
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_configs_dir, 'launch', 'ntrip_client_launch.py')
        ),
        condition=UnlessCondition(is_playback)
    )

    # --- 4. PROCESSING NODES (Always run, but use sim_time) ---

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        # Pass use_sim_time to the included launch file
        launch_arguments={
            'config_file': fast_lio_config,
            'use_sim_time': is_playback,
            "rviz":"false",
        }.items()
    )

    geotagger_node = Node(
        package='my_configs',
        executable='measurement_geotagger',
        name='measurement_geotagger',
        output='screen',
        parameters=[geotagger_config, use_sim_time_param] # Critical for sync during playback
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': LaunchConfiguration('playback')}],
        output='screen'
    )

    return LaunchDescription([
        record_bag_arg,
        playback_arg,
        bag_file_arg,
        bag_recorder,
        bag_player,
        livox_driver_node,
        ublox_node,
        keithley_launch,
        ntrip_launch,
        fast_lio_launch,
        geotagger_node,
        rviz_node,
    ])