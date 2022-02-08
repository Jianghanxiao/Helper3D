import open3d as o3d
import numpy as np
import json
 
DATA_PATH = "/Users/shawn/Desktop/3DHelper/test.json"

if __name__ == "__main__":
    with open(DATA_PATH) as f:
        points = json.load(f)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    o3d.visualization.draw_geometries([pcd])