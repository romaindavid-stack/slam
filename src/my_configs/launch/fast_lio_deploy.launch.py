import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import socket
import time


testing_mode: bool = True


def set_working_mode() -> tuple[bool, bool]:
    is_laptop: bool
    is_jetson: bool

    hostname: str = socket.gethostname()
    if testing_mode:
        is_laptop = True
        is_jetson = True
    else:
        is_laptop = "thinkpad" in hostname.lower()
        is_jetson = "jetson" in hostname.lower()

    if not is_laptop and not is_jetson:
        print(f"ERROR! Couldnt resolve device type (laptop or jetson). Your hostname is {hostname}, change the main launch file so you are recognized as you need.")
        sys.exit(1)
    return is_laptop, is_jetson


def set_record_command(bag_name: str, is_laptop: bool, is_jetson: bool) -> list[str]:
    # Base topics recorded in every situation
    recorded_topics = [
        'ros2', 'bag', 'record', '-o', bag_name,
        '/ublox_gps_node/fix', 
        '/Odometry'
    ]

    if is_jetson:
        # Add raw sensor data usually only available on the Jetson
        recorded_topics.extend([
            '/livox/lidar',
            '/livox/imu'
        ])

    if is_laptop:
        # Add measurement data if the laptop is running the drivers
        recorded_topics.extend([
            '/keithley/measurement', 
            '/keithley/geotagged_marker'
        ])
    
    return recorded_topics




def generate_launch_description():
    
    # --- 0. ARGUMENTS & CONFIG ---
    if not os.path.exists('bags'):
        os.makedirs('bags')

    record_bag_arg = DeclareLaunchArgument('record', default_value='false')
    playback_arg = DeclareLaunchArgument('playback', default_value='false')
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    # Path to the bag you want to play back (only used if playback:=true)
    bag_file_arg = DeclareLaunchArgument('bag_file', default_value='')
    rate_arg = DeclareLaunchArgument('rate', default_value='1.0')
    sensor_arg = DeclareLaunchArgument('sensors', default_value='true')
    long_stick = DeclareLaunchArgument('long_stick', default_value='false')

    # Shortcuts for conditions and parameters
    is_playback = LaunchConfiguration('playback')
    is_recording = LaunchConfiguration('record')
    play_rate = LaunchConfiguration('rate')
    run_slam = LaunchConfiguration('slam')
    run_sensors = LaunchConfiguration('sensors')
    is_long_stick = LaunchConfiguration('long_stick')


    run_sensor = IfCondition(
        PythonExpression([
            "'", run_sensors, "' == 'true' and '", is_playback, "' == 'false'"
        ])
    )

    # This ensures nodes use the bag's clock during playback
    use_sim_time_param: dict[str, LaunchConfiguration] = {'use_sim_time': is_playback}   


    entities: list[LaunchDescriptionEntity] = [
        record_bag_arg,
        playback_arg,
        bag_file_arg,
        rate_arg,
        slam_arg,
        sensor_arg,
        long_stick,
    ]
    is_laptop, is_jetson = set_working_mode()

    # --- 1. RECORDER & PLAYER ---
    bag_name = "bags/slam_run_" + time.strftime("%Y_%m_%d-%H_%M_%S")
    record_command = set_record_command(bag_name, is_laptop, is_jetson)
    bag_recorder = ExecuteProcess(
        condition=IfCondition(is_recording),
        cmd=record_command,
        output='screen'
    )
    entities.append(bag_recorder)
    bag_player = ExecuteProcess(
        cmd=[[
            'ros2 bag play ', 
            LaunchConfiguration('bag_file'), 
            ' --clock',
            ' --rate ', play_rate,
            ' --remap /keithley/geotagged_marker:=/keithley/old_marker /Odometry:=/Odometry_old',
        ]],
        shell=True,
        output='screen',
        condition=IfCondition(is_playback)
    )
    entities.append(bag_player)


    # --- 2. PATHS ---
    my_configs_dir = get_package_share_directory('my_configs')
    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_config = os.path.join(my_configs_dir, 'config', 'mid360.yaml')  # 'mid360.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_configs'), 'rviz', 'geotagger.rviz')
    geotagger_config = os.path.join(get_package_share_directory('my_configs'), 'config', 'geotagger.yaml')
    
    # --- 3. NODES ---

    if is_jetson:
        livox_driver_node = Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            parameters=[{
                "user_config_path": os.path.join(my_configs_dir, 'config', 'MID360_config.json'),
                "xfer_format": 1, "publish_freq": 10.0, "frame_id": 'livox_frame'
            }],
            condition=run_sensor,
            output='screen'
        )
        entities.append(livox_driver_node)
    

        ublox_node = Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            parameters=[os.path.join(my_configs_dir, 'config', 'zed_f9p.yaml')],
            condition=run_sensor,
            output='screen'
        )
        entities.append(ublox_node)

        ntrip_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_configs_dir, 'launch', 'ntrip_client_launch.py')
            ),
            condition=run_sensor,
        )
        entities.append(ntrip_launch)

        fast_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
            ),
            # Pass use_sim_time to the included launch file
            launch_arguments={
                'config_file': fast_lio_config,
                'use_sim_time': is_playback,
                "rviz":"false",  # false  # true
            }.items(),
            condition=IfCondition(run_slam),
        )
        entities.append(fast_lio_launch)
    
    if is_laptop:
            
        keithley_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('keithley_dmm'), 'launch', 'keithley_yaml_launch.py')
            ),
            condition=run_sensor,
        )
        entities.append(keithley_launch)

        geotagger_node = Node(
            package='my_configs',
            executable='measurement_geotagger',
            name='measurement_geotagger',
            output='screen',
            parameters=[geotagger_config, use_sim_time_param, {'is_long_stick': is_long_stick}], # Critical for sync during playback
            condition=IfCondition(run_slam),
        )
        entities.append(geotagger_node)

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('playback')}],
            condition=IfCondition(run_slam),
            output='screen'
        )
        entities.append(rviz_node)


    return LaunchDescription(entities)