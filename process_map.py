import open3d as o3d
import numpy as np

# 1. PCD TO PYTHON (Load)
pcd = o3d.io.read_point_cloud("maps/laser_map_final.pcd")

# 2. CONVERT TO NUMPY (The "Math" Zone)
# These are just standard Nx3 arrays now
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)


mask = points[:, 0]+points[:, 2] > -7
print(points.shape)
points = points[mask]
print(points.shape)
#colors = np.array([[0, 0, 255]])

# -----------------------------

# 3. BACK TO OBJECT (Reconstruct)
new_pcd = o3d.geometry.PointCloud()
new_pcd.points = o3d.utility.Vector3dVector(points)
new_pcd.colors = o3d.utility.Vector3dVector(colors)

# 4. SAVE AGAIN
o3d.io.write_point_cloud("reprocessed.pcd", new_pcd)