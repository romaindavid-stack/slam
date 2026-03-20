import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Finde den Pfad, wo ROS2 deine config.yaml installiert hat
    pkg_share = get_package_share_directory('keithley_dmm')
    config_file_path = os.path.join(pkg_share, 'config', 'keithley_dmm_config.yaml')

    print(f"Lade Konfiguration von: {config_file_path}")

    # 2. Starte den Node und übergib ihm NUR den Pfad zur Datei
    return LaunchDescription([
        Node(
            package='keithley_dmm',
            executable='keithley_dmm_node',
            name='keithley_dmm_node',
            output='screen',
            # Das ist der wichtige Teil: Wir geben ihm direkt die YAML-Datei!
            parameters=[config_file_path]
        )
    ])