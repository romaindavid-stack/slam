from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.1.51',
        description='IP address of the Keithley DMM'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5025',
        description='Port number for Keithley DMM communication'
    )
    
    measurement_type_arg = DeclareLaunchArgument(
        'measurement_type',
        default_value='voltage',
        description='Type of measurement: voltage, current, resistance, temperature'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='1.0',
        description='Sampling rate in Hz'
    )
    
    measurement_range_arg = DeclareLaunchArgument(
        'measurement_range',
        default_value='auto',
        description='Measurement range (auto or specific value)'
    )
    
    high_speed_arg = DeclareLaunchArgument(
        'high_speed_mode',
        default_value='true',
        description='Enable high speed sampling mode'
    )
    
    # Create node
    keithley_node = Node(
        package='keithley_dmm',
        executable='keithley_dmm_node',
        name='keithley_dmm_node',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'port': LaunchConfiguration('port'),
            'measurement_type': LaunchConfiguration('measurement_type'),
            'measurement_range': LaunchConfiguration('measurement_range'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'high_speed_mode': LaunchConfiguration('high_speed_mode'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        ip_address_arg,
        port_arg,
        measurement_type_arg,
        sample_rate_arg,
        measurement_range_arg,
        high_speed_arg,
        keithley_node
    ])
