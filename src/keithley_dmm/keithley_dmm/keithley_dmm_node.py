#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Temperature
import socket
import time
import threading
from typing import Optional

class KeithleyDMMNode(Node):
    """ROS2 Node for Keithley DMM6500/DMM6510 Digital Multimeter"""
    
    def __init__(self):
        super().__init__('keithley_dmm_node')
        
        # Declare parameters
        self.declare_parameter('ip_address', '169.254.12.66')  # Default IP address
        self.declare_parameter('port', 5025)  # Default port
        self.declare_parameter('measurement_type', 'voltage')  # voltage, current, resistance, temperature
        self.declare_parameter('measurement_range', 'auto')
        self.declare_parameter('sample_rate', 1.0)  # Hz
        self.declare_parameter('nplc', 1.0)  # Number of power line cycles
        self.declare_parameter('high_speed_mode', True)  # Enable high speed sampling
        
        # Get parameters
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.measurement_type = self.get_parameter('measurement_type').get_parameter_value().string_value
        self.measurement_range = self.get_parameter('measurement_range').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        self.nplc = self.get_parameter('nplc').get_parameter_value().double_value
        self.high_speed_mode = self.get_parameter('high_speed_mode').get_parameter_value().bool_value
        
        # Initialize socket connection
        self.connection: Optional[socket.socket] = None
        
        # Publishers
        self.measurement_pub = self.create_publisher(Float64, 'keithley/measurement', 10)
        self.status_pub = self.create_publisher(String, 'keithley/status', 10)
        self.temperature_pub = self.create_publisher(Temperature, 'keithley/temperature', 10)
        
        # Timer for periodic measurements
        self.measurement_timer = self.create_timer(1.0 / self.sample_rate, self.measure_callback)
        
        # Status variables
        self.is_connected = False
        self.last_measurement = 0.0
        
        # Connect to instrument
        self.connect_instrument()
        
        self.get_logger().info(f'Keithley DMM Node initialized for {self.measurement_type} measurements')

    def connect_instrument(self):
        """Connect to the Keithley instrument"""
        try:
            # Create socket connection
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.settimeout(10)  # 10 second timeout
            
            # Connect to the instrument
            self.get_logger().info(f'Connecting to Keithley at {self.ip_address}:{self.port}')
            self.connection.connect((self.ip_address, self.port))
            
            # Test connection with IDN query
            self.send_command('*IDN?')
            idn = self.read_response()
            self.get_logger().info(f'Connected to: {idn.strip()}')
            
            # Reset instrument
            self.send_command('*RST')
            time.sleep(1)
            
            # Configure measurement
            self.configure_measurement()
            
            self.is_connected = True
            self.publish_status("Connected")
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to instrument: {str(e)}')
            self.is_connected = False
            self.publish_status(f"Connection failed: {str(e)}")
            if self.connection:
                try:
                    self.connection.close()
                except:
                    pass
                self.connection = None

    def send_command(self, command: str):
        """Send a command to the instrument"""
        if not self.connection:
            raise RuntimeError("Not connected to instrument")
        
        if not command.endswith('\n'):
            command += '\n'
        
        self.connection.send(command.encode())

    def read_response(self) -> str:
        """Read response from the instrument"""
        if not self.connection:
            raise RuntimeError("Not connected to instrument")
        
        return self.connection.recv(1024).decode().strip()

    def configure_measurement(self):
        """Configure the measurement type and parameters"""
        if not self.connection:
            return
            
        try:
            # Configure based on measurement type
            if self.measurement_type.lower() == 'voltage':
                self.send_command(':SENS:FUNC "VOLT:DC"')
                
                if self.measurement_range == 'auto':
                    self.send_command(':SENS:VOLT:DC:RANG:AUTO ON')
                else:
                    self.send_command(f':SENS:VOLT:DC:RANG {self.measurement_range}')
                
                self.send_command(f':SENS:VOLT:DC:NPLC {self.nplc}')
                
                if self.high_speed_mode:
                    # High speed settings from your original code
                    self.send_command(':SENS:VOLT:DC:AZER ON')
                    self.send_command(':SENS:VOLT:DC:AVER:STAT OFF')
                    self.get_logger().info("High speed sampling mode enabled")
                
            elif self.measurement_type.lower() == 'current':
                self.send_command(':SENS:FUNC "CURR:DC"')
                
                if self.measurement_range == 'auto':
                    self.send_command(':SENS:CURR:DC:RANG:AUTO ON')
                else:
                    self.send_command(f':SENS:CURR:DC:RANG {self.measurement_range}')
                
                self.send_command(f':SENS:CURR:DC:NPLC {self.nplc}')
                
            elif self.measurement_type.lower() == 'resistance':
                self.send_command(':SENS:FUNC "RES"')
                
                if self.measurement_range == 'auto':
                    self.send_command(':SENS:RES:RANG:AUTO ON')
                else:
                    self.send_command(f':SENS:RES:RANG {self.measurement_range}')
                
                self.send_command(f':SENS:RES:NPLC {self.nplc}')
                
            elif self.measurement_type.lower() == 'temperature':
                self.send_command(':SENS:FUNC "TEMP"')
                self.send_command(':SENS:TEMP:TRAN TC')  # Thermocouple
                self.send_command(':SENS:TEMP:TC:TYPE K')  # K-type thermocouple
                self.send_command(f':SENS:TEMP:NPLC {self.nplc}')
            
            self.get_logger().info(f'Configured for {self.measurement_type} measurements')
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure measurement: {str(e)}')

    def measure_callback(self):
        """Timer callback to perform measurements"""
        if not self.is_connected or not self.connection:
            return
            
        try:
            # Take measurement
            self.send_command('READ?')
            measurement_str = self.read_response()
            measurement = float(measurement_str)
            self.last_measurement = measurement
            
            # Publish measurement
            msg = Float64()
            msg.data = measurement
            self.measurement_pub.publish(msg)
            
            # If temperature measurement, also publish as Temperature message
            if self.measurement_type.lower() == 'temperature':
                temp_msg = Temperature()
                temp_msg.header.stamp = self.get_clock().now().to_msg()
                temp_msg.header.frame_id = 'keithley_dmm'
                temp_msg.temperature = measurement
                temp_msg.variance = 0.0  # You may want to calculate this based on your needs
                self.temperature_pub.publish(temp_msg)
            
            # Log periodically (every 10 measurements)
            if hasattr(self, 'measurement_count'):
                self.measurement_count += 1
            else:
                self.measurement_count = 1
                
            if self.measurement_count % 10 == 0:
                unit = self.get_unit_string()
                self.get_logger().info(f'Measurement: {measurement:.6f} {unit}')
            
        except Exception as e:
            self.get_logger().error(f'Measurement failed: {str(e)}')
            self.publish_status(f"Measurement error: {str(e)}")

    def get_unit_string(self) -> str:
        """Get the appropriate unit string for the measurement type"""
        units = {
            'voltage': 'V',
            'current': 'A',
            'resistance': 'Ω',
            'temperature': '°C'
        }
        return units.get(self.measurement_type.lower(), 'units')

    def publish_status(self, status: str):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        if self.connection:
            try:
                self.connection.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeithleyDMMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()