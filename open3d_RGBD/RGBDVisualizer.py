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
    getConventionTransform
)

source = 'shape2motion'
model = 'lamp_0060'
index = '1-1-2'
DATA_PATH = f"/Users/apple/Desktop/3DHelper/data/{model}/"


def getMotion(motion):
    # Visualize 3D BBX
    min_bound_raw = motion['3dbbx']['min']
    max_bound_raw = motion['3dbbx']['max']
    min_bound = np.array(
        [min_bound_raw['x'], min_bound_raw['y'], min_bound_raw['z']])
    max_bound = np.array(
        [max_bound_raw['x'], max_bound_raw['y'], max_bound_raw['z']])
    bbx = BBX(min_bound, max_bound, color=[0.8, 0.2, 0])
    bbx.transform(convention_matrix)
    bbx = bbx.getMesh()
    # Visualize motion axis (still need consider translation visualization)
    origin_raw = motion['origin']
    origin = np.array([origin_raw['x'], origin_raw['y'], origin_raw['z']])
    axis_raw = motion['axis']
    axis = np.array([axis_raw['x'], axis_raw['y'], axis_raw['z']])
    arrow = get_arrow(origin=origin-axis, vec=3*axis, color=[0, 1, 1])
    arrow.transform(convention_matrix)

    return [bbx, arrow]


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

    # Calculate the convention matrix
    convention_matrix = getConventionTransform(source)

    # Visualize the world coordinate
    world = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # Visualize the camera coordinate
    camera = getCamera(transformation, fx, fy, cx, cy)

    # Visualize annotation (Just one motion for testing)
    motions = []
    motion_num = len(annotation['motions'])
    for motion_index in range(motion_num):
        motions += getMotion(annotation['motions'][motion_index])

    # Final Visualization
    o3d.visualization.draw_geometries([pcd, world] + camera + motions)
