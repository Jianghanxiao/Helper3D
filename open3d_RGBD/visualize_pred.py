import open3d as o3d
import numpy as np
import json
import math
import time

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

model = '45194'
index = '0-0'
mask_index = '0'
DATA_PATH = f"/Users/shawn/Desktop/3DHelper/data/{model}/"
current_origin =  np.array([
                0.7403511818922486,
                -0.4674381764105489,
                -3.547396477962706
            ])
current_axis = np.array([
                0,
                -0.575936130687482,
                -0.8174946931746598
            ])
motion_tupe = 'rotation'
width = 256
height = 256
fov = 50



def get_pcd_from_rgbd_mask(color_img_path, depth_img_path, mask_img_path, fx, fy, cx, cy, background_filter=None):
    # Self-designed with depth unit transform and background filter
    # Calculate the projection matrix by hand; Add manual background filter
    color_raw = np.array(o3d.io.read_image(color_img_path)) / 255
    # Convert the unit to meter
    depth_raw = np.array(o3d.io.read_image(depth_img_path)) / 1000
    mask_raw = np.array(o3d.io.read_image(mask_img_path))
    # For static part, the mask_raw[, , 1] = 0 & alpha = 255
    # For moving part, the mask_raw[, , 1] = 255 & alpha = 255

    height, width = np.shape(depth_raw)
    points_static = []
    colors_static = []

    for y in range(height):
        for x in range(width):
            if depth_raw[y][x] < 1 or mask_raw[y][x][1] == 255 or mask_raw[y][x][3] == 0:
                continue
            colors_static.append(color_raw[y][x])
            points_static.append([(x - cx) * (depth_raw[y][x] / fx),
                           -(y - cy) * (depth_raw[y][x] / fy), -depth_raw[y][x]])

    pcd_static = o3d.geometry.PointCloud()
    pcd_static.points = o3d.utility.Vector3dVector(np.array(points_static))
    pcd_static.colors = o3d.utility.Vector3dVector(np.array(colors_static)[:, 0:3])

    points_moving = []
    colors_moving = []

    for y in range(height):
        for x in range(width):
            if depth_raw[y][x] < 1 or mask_raw[y][x][1] == 0 or mask_raw[y][x][3] == 0:
                continue
            colors_moving.append(color_raw[y][x])
            points_moving.append([(x - cx) * (depth_raw[y][x] / fx),
                           -(y - cy) * (depth_raw[y][x] / fy), -depth_raw[y][x]])

    pcd_moving = o3d.geometry.PointCloud()
    pcd_moving.points = o3d.utility.Vector3dVector(np.array(points_moving))
    pcd_moving.colors = o3d.utility.Vector3dVector(np.array(colors_moving)[:, 0:3])

    return (pcd_static, pcd_moving)

if __name__ == "__main__":
    color_img = f'{DATA_PATH}origin/{model}-{index}.png'
    depth_img = f'{DATA_PATH}depth/{model}-{index}_d.png'
    mask_img = f'{DATA_PATH}mask/{model}-{index}_{mask_index}.png'
    # Read the annotation
    annotation_file = open(
        f'{DATA_PATH}origin_annotation/{model}-{index}.json')
    annotation = json.load(annotation_file)
    # Read camera intrinsics
    img_height = height
    img_width = width
    FOV = fov
    # The width and height are the same
    fy, fx = getFocalLength(FOV / 180 * math.pi, img_height, img_width)
    cy = img_height / 2
    cx = img_width / 2
    # get the pc without background
    pcd_static, pcd_moving = get_pcd_from_rgbd_mask(color_img, depth_img, mask_img,
                            fx, fy, cx, cy, 1000)

    # Visualize the camera coordinate
    transformation = np.eye(4)
    camera = getCamera(transformation, fx, fy, cx, cy)

    # Visualize the axis for the rotation or translation
    axis_point = current_origin + current_axis
    arrow = get_arrow(origin=current_origin-current_axis, end=axis_point, color=[0, 1, 1])

    # Construct the small rotation
    trans1 = np.eye(4)
    trans1[0:3, 3] = -current_origin
    # Get the rotation matrix based on rotating around a axis start from origin point
    rot = np.array([[  0.9800666,  0.1624111, -0.1144208, 0],[-0.1624111,  0.9866785,  0.0093851, 0],[0.1144208,  0.0093851,  0.9933881, 0], [0, 0, 0, 1]])
    trans2 = np.eye(4)
    trans2[0:3, 3] = current_origin
    little_trans = np.matmul(trans2, np.matmul(rot, trans1))

    # Visualize the geometry
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_static)
    vis.add_geometry(pcd_moving)
    vis.add_geometry(arrow)
    vis.add_geometry(camera[0])
    vis.add_geometry(camera[1])
    # static_stuff = [pcd_static, pcd_moving]
    for i in range(100):
        pcd_moving.transform(little_trans)
        vis.update_geometry(pcd_moving)
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()
    # Final Visualization
    # o3d.visualization.draw_geometries([pcd_moving, arrow] + camera)

    # o3d.visualization.draw_geometries([pcd, world])
