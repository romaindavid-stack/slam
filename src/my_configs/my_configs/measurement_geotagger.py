import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from collections import deque
import numpy as np
import math

class MeasurementGeotagger(Node):
    def __init__(self):
        super().__init__('measurement_geotagger')

        # --- CONFIGURATION ---
        self.max_odom_buffer_size = 200
        self.max_interpolation_gap = 0.5 # Max seconds between odom frames to allow interpolation
        
        # LEVER ARM OFFSET (in meters)
        # Coordinates in the Robot/Odom frame:
        # x: forward/backward, y: left/right, z: up/down
        # Example: Sensor is 0.5m behind and 0.2m below the LiDAR center
        self.lever_arm = np.array([-0.5, 0.0, -0.2])

        # --- BUFFERS ---
        # We store odom as a list of dictionaries for easy time-searching
        self.odom_buffer = deque(maxlen=self.max_odom_buffer_size)
        # Store measurements that are waiting for a "future" odom frame to arrive
        self.measurements_waiting_room = deque()

        # --- ROS INTERFACE ---
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry', 
            self.odom_callback, 
            10
        )
        self.meas_sub = self.create_subscription(
            Float64, 
            '/keithley/measurement', 
            self.measurement_callback, 
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/keithley/geotagged_pose', 
            10
        )

        self.get_logger().info("Geotagger Node Started.")
        self.get_logger().info(f"Lever arm offset configured: {self.lever_arm}")

    def get_timestamp(self, msg_header):
        return msg_header.stamp.sec + msg_header.stamp.nanosec * 1e-9

    def measurement_callback(self, msg):
        """Called when Keithley publishes a value."""
        # Use the ROS clock time of receipt
        t_meas = self.get_clock().now().nanoseconds * 1e-9
        self.measurements_waiting_room.append({'time': t_meas, 'value': msg.data})
        
        # Check if we can process this immediately
        self.process_queue()

    def odom_callback(self, msg):
        """Called when Odometry (from Fast-LIO) arrives."""
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
            return

        while self.measurements_waiting_room:
            t_target = self.measurements_waiting_room[0]['time']
            
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

    def interpolate_and_publish(self, meas, prev, next_):
        dt = next_['time'] - prev['time']
        
        # Sanity check for data gaps
        if dt > self.max_interpolation_gap or dt <= 0:
            self.get_logger().error(f"Odom gap too wide ({dt:.2f}s). Skipping measurement.")
            return

        # Linear interpolation factor (0.0 to 1.0)
        alpha = (meas['time'] - prev['time']) / dt
        
        # Interpolate Position
        interp_pos = prev['pos'] + alpha * (next_['pos'] - prev['pos'])
        
        # Interpolate Rotation (nlerp)
        q1 = prev['quat']
        q2 = next_['quat']
        
        interp_q = [
            q1.x + alpha * (q2.x - q1.x),
            q1.y + alpha * (q2.y - q1.y),
            q1.z + alpha * (q2.z - q1.z),
            q1.w + alpha * (q2.w - q1.w)
        ]
        # Re-normalize
        norm = math.sqrt(sum(i*i for i in interp_q))
        interp_q = [i/norm for i in interp_q]

        # --- ROTATING THE LEVER ARM ---
        rotated_offset = self.rotate_vector(self.lever_arm, interp_q)
        final_pos = interp_pos + rotated_offset

        # --- PUBLISH ---
        out_msg = PoseStamped()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = prev['frame_id']
        
        out_msg.pose.position.x = final_pos[0]
        out_msg.pose.position.y = final_pos[1]
        out_msg.pose.position.z = final_pos[2]
        
        out_msg.pose.orientation.x = interp_q[0]
        out_msg.pose.orientation.y = interp_q[1]
        out_msg.pose.orientation.z = interp_q[2]
        out_msg.pose.orientation.w = interp_q[3]

        self.pose_pub.publish(out_msg)

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
        rclpy.shutdown()

if __name__ == '__main__':
    main()