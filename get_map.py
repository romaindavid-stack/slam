import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os
import csv


import numpy as np

def specific_post_processing(points):
    # 1. View the structured array as a simple float32 array
    # We use .view('<f4') because your error showed float32 ('<f4')
    # and reshape it to (-1, 3) to represent [N, 3] coordinates.
    numeric_points = points.view('<f4').reshape(-1, 3)

    # 2. Calculate the Euclidean distance (norm)
    distances = np.linalg.norm(numeric_points, axis=1)

    # 3. Create a mask based on the distance threshold
    mask = distances < 10.0

    # 4. Apply the mask back to the ORIGINAL structured points 
    # so you keep the 'x', 'y', 'z' field structure
    filtered_points = points[mask]
    
    return filtered_points



def open_csv():
    file_path=os.path.expanduser("~/slam/maps/markers.csv")
    points = []
    colors = []
    
    try:
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip the header row ["X", "Y", "Z", "r", "g", "b"]
            
            for row in reader:
                # Convert strings from CSV back to floats
                x, y, z = float(row[0]), float(row[1]), float(row[2])
                r, g, b = float(row[3]), float(row[4]), float(row[5])
                
                points.append([x, y, z])
                colors.append([r, g, b])
                
    except FileNotFoundError:
        print(f"File {file_path} not found. Returning empty lists.")
        
    return points, colors

class LaserMapGrabber(Node):
    def __init__(self):
        super().__init__('laser_map_grabber')
        # Using the exact topic and QoS settings from your RViz config
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/Laser_map', 
            self.listener_callback,
            qos)
        
        self.get_logger().info('Waiting for /Laser_map message...')
        self.get_logger().info('Note: Fast-LIO usually publishes this every few seconds.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received map with {msg.width * msg.height} points. Saving...')
        
        # Convert ROS2 PointCloud2 to numpy
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        points = points.view(points.dtype[0]).reshape(-1, 3)
        
        if len(points) > 0:
            points = specific_post_processing(points)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd = pcd.voxel_down_sample(voxel_size=0.05)

            # 3. Statistical Outlier Removal
            # nb_neighbors: Number of neighbors to analyze for each point
            # std_ratio: Standard deviation multiplier (Lower = More Aggressive cleaning)
            pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            # Save exactly what was received
            save_path = os.path.expanduser("maps/laser_map_final.pcd")
            save_path2 = os.path.expanduser("maps/colored_progress.pcd")
            o3d.io.write_point_cloud(save_path, pcd)
            
            self.get_logger().info(f'SUCCESS! Global map saved to {save_path}')

            points, colors = open_csv()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # 4. Save to PCD
            o3d.io.write_point_cloud(save_path2, pcd)
            # We only need one map, so we can exit now
            raise SystemExit 

def main(args=None):
    rclpy.init(args=args)
    node = LaserMapGrabber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()