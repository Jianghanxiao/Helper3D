import h5py
import open3d as o3d
import numpy as np

from src import (
    get_pcd_from_array,
    get_arrow,
)

file_path = "/Users/shawn/Desktop/3DHelper/data/val.h5"

if __name__ == "__main__":
    results = h5py.File(file_path)

    instances = results.keys()
    for instance in instances:
        if instance == "48686-0-3-3":
            print(instance)
            result = results[instance]
            camcs_per_point = result["camcs_per_point"][:]
            origin = np.array([-0.18494093,  0.01504402, -3.69099359])
            axis = np.array([0.99839507, -0.03667576,  0.04315296])
            axis_point = origin + axis
            arrow = get_arrow(origin=origin, end=axis_point, color=[0, 1, 1])
            pcd = get_pcd_from_array(camcs_per_point)

            o3d.visualization.draw_geometries([pcd, arrow])
            break
