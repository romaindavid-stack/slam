import csv
from random import randint
import time
from typing import Literal

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from collections import deque
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class MeasurementGeotagger(Node):
    def __init__(self):
        super().__init__('measurement_geotagger')
        self.debugging = 0  # False  # True


        # --- DECLARE PARAMETERS (With Defaults) ---
        self.declare_parameter('filter_start_sec', 0.0)
        self.declare_parameter('filter_end_sec', -1.0)
        self.declare_parameter('measurement_step', 1)
        self.declare_parameter('min_volt', -0.5)
        self.declare_parameter('max_volt', 0.5)
        self.declare_parameter('save', False)
        # LEVER ARM OFFSET (in meters)
        self.declare_parameter('lever_arm', [0.14, 0.0, 0.85])
        # --- LOAD PARAMETERS ---
        self.start_window = self.get_parameter('filter_start_sec').value
        self.end_window = self.get_parameter('filter_end_sec').value
        self.step = self.get_parameter('measurement_step').value
        self.min_volt = self.get_parameter('min_volt').value
        self.max_volt = self.get_parameter('max_volt').value
        self.lever_arm = np.array(self.get_parameter('lever_arm').value)
        self.save = self.get_parameter('save').value


        self.print(f"\n\n\nmeasurement geotagger well and alive\nStarting measurements from second {self.start_window} to second {self.end_window}\n\n\n")

        # --- CONFIGURATION ---
        self.max_odom_buffer_size = 200
        self.max_interpolation_gap = 0.5 # Max seconds between odom frames to allow interpolation
        

        # --- BUFFERS ---
        # We store odom as a list of dictionaries for easy time-searching
        self.odom_buffer = deque(maxlen=self.max_odom_buffer_size)
        # Store measurements that are waiting for a "future" odom frame to arrive
        self.measurements_waiting_room = deque()
        self.marker_id = 0
        self.first_msg_time = None
        self.meas_count = 0


        # --- ROS INTERFACE ---
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/Odometry', 
            self.odom_callback, 
            10
        )
        self.meas_sub = self.create_subscription(
            Float64, 
            '/keithley/measurement', 
            self.measurement_callback, 
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/keithley/geotagged_marker', 10)

        self.print(f"Geotagger Started. Gradient: Green -> Yellow -> Orange -> Red ({self.min_volt}V to {self.max_volt}V)")
        self.print(f"Lever arm offset configured: {self.lever_arm}")

        # files
        self.save_path = 'maps/markers.csv'  # f'maps/markers_{time.strftime("%Y_%m_%d-%H_%M_%S")}.csv'
        if self.save:
            with open(self.save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["X", "Y", "Z", "r", "g", "b"]) # Header

    def print(self, msg, prio=False):
        if self.debugging or prio:
            self.get_logger().info(str(msg))

    def get_timestamp(self, msg_header):
        return msg_header.stamp.sec + msg_header.stamp.nanosec * 1e-9

    def measurement_callback(self, msg):
        """Called when Keithley publishes a value."""
        # Use the ROS clock time of receipt
        self.print("received measurement")
        t_now = self.get_clock().now().nanoseconds * 1e-9
        # 2. Track mission start time
        if self.first_msg_time is None:
            self.first_msg_time = t_now
            return
        elapsed = t_now - self.first_msg_time
        # 3. Apply Time Filter
        if elapsed < self.start_window or (elapsed > self.end_window and self.end_window != -1):  # -1: no end time
            return
        # 4. Apply Downsampling (Measurement Step)
        self.meas_count += 1
        if self.meas_count % self.step != 0:
            return

        self.measurements_waiting_room.append({'time': t_now, 'value': msg.data})
        self.print(msg.data)

        
        # Check if we can process this immediately
        self.process_queue()

    def odom_callback(self, msg):
        """Called when Odometry (from Fast-LIO) arrives."""
        self.print("received odom")
        t_odom = self.get_timestamp(msg.header)
        
        # Store minimal data needed for interpolation
        self.odom_buffer.append({
            'time': t_odom,
            'pos': np.array([msg.pose.pose.position.x, 
                             msg.pose.pose.position.y, 
                             msg.pose.pose.position.z]),
            'quat': msg.pose.pose.orientation,
            'frame_id': msg.header.frame_id
        })
        
        # Every time a new "future" odom arrives, check if we can process waiting measurements
        self.process_queue()

    def process_queue(self):
        """Attempts to match waiting measurements with odom flanks."""
        if len(self.odom_buffer) < 2 or not self.measurements_waiting_room:
            self.print("missing odom or measurements")
            return

        while self.measurements_waiting_room:
            t_target = self.measurements_waiting_room[0]['time']
            self.print(f"t_target: {t_target}")
            self.print(f"odom buffer: {self.odom_buffer[0]['time']} to {self.odom_buffer[-1]['time']}")
            
            # 1. Check if the measurement is too old for our buffer
            if t_target < self.odom_buffer[0]['time']:
                self.get_logger().warn("Measurement too old for buffer, discarding.")
                self.measurements_waiting_room.popleft()
                continue

            # 2. Check if the 'future' odom has arrived yet
            if t_target > self.odom_buffer[-1]['time']:
                # Still waiting for a newer odom frame. Stop loop and wait for next odom_callback.
                break

            # 3. Find the 'flanks' (the messages exactly before and after the measurement)
            prev_frame = None
            next_frame = None
            for i in range(len(self.odom_buffer) - 1):
                if self.odom_buffer[i]['time'] <= t_target <= self.odom_buffer[i+1]['time']:
                    prev_frame = self.odom_buffer[i]
                    next_frame = self.odom_buffer[i+1]
                    break
            
            if prev_frame and next_frame:
                meas = self.measurements_waiting_room.popleft()
                self.interpolate_and_publish(meas, prev_frame, next_frame)
            else:
                # If we are here and t_target < odom_buffer[-1], it means there's a gap 
                # in our odom buffer we can't bridge.
                break

	
    def get_color(self, value):
        """Maps a value to a smooth Green -> Yellow -> Orange -> Red gradient."""
        # Normalize value between 0 and 1
        norm = (value - self.min_volt) / (self.max_volt - self.min_volt)
        norm = max(0.0, min(1.0, norm)) # Clamp
        norm = 1-norm  # small is bad
        # Gradient Logic:
        # 0.0 -> Green (0, 1, 0)
        # 0.33 -> Yellow (1, 1, 0)
        # 0.66 -> Orange (1, 0.5, 0)
        # 1.0 -> Red (1, 0, 0)
        
        if norm < 0.33:
            # Green to Yellow: Increase Red
            r = norm / 0.33
            g = 1.0
            b = 0.0
        elif norm < 0.66:
            # Yellow to Orange: Decrease Green to 0.5
            r = 1.0
            g = 1.0 - 0.5 * ((norm - 0.33) / 0.33)
            b = 0.0
        else:
            # Orange to Red: Decrease Green to 0.0
            r = 1.0
            g = 0.5 * (1.0 - (norm - 0.66) / 0.34)
            b = 0.0
            
        return r, g, b

    def interpolate_and_publish(self, meas, previous_odometry, next_odometry):
        dt = next_odometry['time'] - previous_odometry['time']
        
        # Sanity check for data gaps
        if dt > self.max_interpolation_gap or dt <= 0:
            self.get_logger().error(f"Odom gap too wide ({dt:.2f}s). Skipping measurement.")
            return

        # Linear interpolation factor (0.0 to 1.0)
        alpha = (meas['time'] - previous_odometry['time']) / dt
        
        # Interpolate Position
        interp_pos = previous_odometry['pos'] + alpha * (next_odometry['pos'] - previous_odometry['pos'])
        
        # # Interpolate Rotation
        rotation_interpolation: Literal["slerp", "nlerp"] = "slerp"
        if rotation_interpolation == "nlerp":
            
            q1 = [previous_odometry['quat'].x, previous_odometry['quat'].y, previous_odometry['quat'].z, previous_odometry['quat'].w]
            q2 = [next_odometry['quat'].x, next_odometry['quat'].y, next_odometry['quat'].z, next_odometry['quat'].w]
        
            interp_q = [q1[i] + alpha * (q2[i] - q1[i]) for i in range(4)]

        elif rotation_interpolation == "slerp":
            # 1. Convert ROS quaternions to lists [x, y, z, w]
            q1_raw = [previous_odometry['quat'].x, previous_odometry['quat'].y, previous_odometry['quat'].z, previous_odometry['quat'].w]
            q2_raw = [next_odometry['quat'].x, next_odometry['quat'].y, next_odometry['quat'].z, next_odometry['quat'].w]

            # 2. Create Scipy Rotation objects
            times = [previous_odometry['time'], next_odometry['time']]
            key_rots = R.from_quat([q1_raw, q2_raw])

            # 3. Perform SLERP
            slerp = Slerp(times, key_rots)
            interp_rot_obj = slerp([meas['time']]) # Interpolate at the measurement time
            interp_q = interp_rot_obj.as_quat()[0] # Get back [x, y, z, w]
            # Re-normalize
            norm = math.sqrt(sum(i*i for i in interp_q))
            interp_q = [i/norm for i in interp_q]

        # --- ROTATING THE LEVER ARM ---
        rotated_offset = self.rotate_vector(self.lever_arm, interp_q)
        final_pos = interp_pos + rotated_offset

        # --- PUBLISH ---
        marker = Marker()
        marker.header.frame_id = previous_odometry['frame_id']
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "keithley_measurements"
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = final_pos[0]
        marker.pose.position.y = final_pos[1]
        marker.pose.position.z = final_pos[2]
        
        marker.scale.x = 0.15 
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # Smooth Color Gradient
        measurement = meas['value']
        if not randint(0, 10):
            self.print(f"measurement value: {measurement}", prio=True)

        r, g, b = self.get_color(measurement)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0 
            
        marker.text = f"{meas['value']:.2f}V"
        
        self.marker_pub.publish(marker)

        if self.save:
            with open(self.save_path, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    marker.pose.position.x, 
                    marker.pose.position.y, 
                    marker.pose.position.z, 
                    r,
                    g,
                    b,
                ])

        self.print("published")

    def rotate_vector(self, v, q):
        """Rotates vector v by quaternion q: v' = v + 2 * q_vec x (q_vec x v + q_w * v)"""
        q_vec = np.array([q[0], q[1], q[2]])
        q_w = q[3]
        
        a = np.cross(q_vec, v) + q_w * v
        b = np.cross(q_vec, a)
        return v + 2.0 * b

def main(args=None):
    rclpy.init(args=args)
    node = MeasurementGeotagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()