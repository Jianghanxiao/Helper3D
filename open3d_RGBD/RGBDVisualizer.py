import open3d as o3d
import numpy as np
import json
import math

from src import (
    getFocalLength,
    get_pcd_from_rgbd,
    get_pcd_from_whole_rgbd,
    get_arrow,
    getCamera,
    BBX,
    getConventionTransform,
    getMotion,
)

model = '7128'
index = '0-2-0'
DATA_PATH = f"/Users/shawn/Desktop/3DHelper/data/{model}_pose/"


if __name__ == "__main__":
    color_img = f'{DATA_PATH}origin/{model}-{index}.png'
    depth_img = f'{DATA_PATH}depth/{model}-{index}_d.png'
    # Read the annotation
    annotation_file = open(
        f'{DATA_PATH}origin_annotation/{model}-{index}.json')
    annotation = json.load(annotation_file)
    # Read camera intrinsics
    img_height = annotation['height']
    img_width = annotation['width']
    FOV = annotation['camera']['intrinsic']['fov']
    # The width and height are the same
    fy, fx = getFocalLength(FOV / 180 * math.pi, img_height, img_width)
    cy = img_height / 2
    cx = img_width / 2
    # get the pc without background
    pcd = get_pcd_from_rgbd(color_img, depth_img,
                            fx, fy, cx, cy, 1000)

    # Deal with the camera pose matrix
    matrix_raw = annotation['camera']['extrinsic']['matrix']
    transformation = np.reshape(matrix_raw, (4, 4)).T
    # Transform the pcd into object coordinate using extrinsic matrix
    pcd.transform(transformation)

    # Visualize the world coordinate
    world = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # Visualize the camera coordinate
    camera = getCamera(transformation, fx, fy, cx, cy)

    # Visualize annotation (The annotation is in camera coordinate)
    motions = []
    motion_num = len(annotation['motions'])
    for motion_index in range(motion_num):
        motions += getMotion(annotation['motions'][motion_index], transformation, state='current')

    # Final Visualization
    o3d.visualization.draw_geometries([pcd, world] + camera + motions)

    # o3d.visualization.draw_geometries([pcd, world])
