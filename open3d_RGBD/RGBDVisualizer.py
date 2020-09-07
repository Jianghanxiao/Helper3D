import open3d as o3d
import numpy as np
import json
import math

from src import (
    getFocalLength,
    get_pcd_from_rgbd,
    get_pcd_from_whole_rgbd
)

model = '7128'
DATA_PATH = f"/Users/apple/Desktop/3DHelper/data/{model}/"

if __name__ == "__main__":
    color_img = f'{DATA_PATH}origin/{model}-0-2-0.png'
    depth_img = f'{DATA_PATH}depth/{model}-0-2-0_d.png'
    # Read the annotation
    annotation_file = open(f'{DATA_PATH}origin_annotation/{model}-0-2-0.json')
    annotation = json.load(annotation_file)
    # Read camera intrinsics
    img_height = annotation['height']
    img_width = annotation['width']
    FOV = annotation['camera']['intrinsic']['fov']
    # The width and height are the same
    fy, fx = getFocalLength(FOV / 180 * math.pi, img_height, img_width)
    # Display the pc without background
    pcd = get_pcd_from_rgbd(color_img, depth_img,
                            fx, fy, img_height / 2, img_height / 2, 1000)
    transformHelper = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0,0,0])
    # Transform the pcd into object coordinate using extrinsic matrix
    matrix_raw = annotation['camera']['extrinsic']['matrix']
    transformation = np.reshape(matrix_raw, (4, 4)).T
    camera_trans = np.eye(4)
    camera_trans[0, 0] = -1
    camera_trans[2, 2] = -1
    transformation = np.dot(camera_trans, transformation)
    pcd.transform(transformation)

    # Load 3dBBx to check if current coordinate is in object coordinate
    min_bound_raw = annotation['motions'][0]['3dbbx']['min']
    max_bound_raw = annotation['motions'][0]['3dbbx']['max']
    min_bound = np.array([min_bound_raw['x'], min_bound_raw['y'], min_bound_raw['z']])
    max_bound = np.array([max_bound_raw['x'], max_bound_raw['y'], max_bound_raw['z']])
    bbx = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    bbx = o3d.geometry.OrientedBoundingBox.create_from_axis_aligned_bounding_box(bbx)
    # transformHelper.transform(transformation1)

    o3d.visualization.draw_geometries([pcd, bbx, transformHelper])
