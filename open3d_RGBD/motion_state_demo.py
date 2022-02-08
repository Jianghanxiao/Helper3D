# This is the script to visualize animation of the predicted motion using open3D

import open3d as o3d
import numpy as np
from numpy.linalg import norm
import argparse

from src import (
    getFocalLength,
    get_pcd_from_rgbd,
    get_pcd_from_whole_rgbd,
    get_pcd_from_rgbd_mask,
    get_arrow,
    getCamera,
    BBX,
    getConventionTransform,
    getMotion,
    axisAngleToRotationMatrix,
)

extrinsic = np.array([
                -0.9799812574061533,
                -1.3877787807814457e-17,
                -0.19908976651916468,
                0,
                -0.16275482759479926,
                0.575936130687482,
                0.8011294773401607,
                0,
                0.11466298978852194,
                0.8174946931746603,
                -0.5644066135367534,
                0,
                0.38804677450198255,
                2.6689343014712645,
                -1.8920640080140574,
                1
            ]).reshape((4, 4), order='F')

def get_parser():

    # Initialize the default path for images
    model = "45194"
    index = "0-0"
    mask_index = "0"
    DATA_PATH = f"/Users/shawn/Desktop/3DHelper/data/{model}/"
    color_img = f"{DATA_PATH}origin/{model}-{index}.png"
    depth_img = f"{DATA_PATH}depth/{model}-{index}_d.png"
    mask_img = f"{DATA_PATH}mask/{model}-{index}_{mask_index}.png"

    # Initialize the default camera intrinsic parameters
    img_width = 256
    img_height = 256
    camera_fov = 50

    # Initialize the default motion parameters
    motion_origin = [0.7403511818922486, -0.4674381764105489, -3.547396477962706]
    motion_axis = [0, -0.575936130687482, -0.8174946931746598]
    motion_type = "rotation"

    parser = argparse.ArgumentParser(
        description="RGBD & Mask & Predicted motion Visualizer option"
    )
    # Parameters for the image path
    parser.add_argument(
        "--rgb_path",
        default=color_img,
        metavar="FILE",
        help="path to the RGB image",
    )
    parser.add_argument(
        "--depth_path",
        default=depth_img,
        metavar="FILE",
        help="path to the depth image",
    )
    parser.add_argument(
        "--mask_path",
        default=mask_img,
        metavar="FILE",
        help="path to the mask image of the moving part",
    )

    # Parameters for the camera intrinsic parameters
    parser.add_argument(
        "--img_wh",
        nargs=2,
        default=[img_width, img_height],
        metavar=("image_width", "image_height"),
        type=int,
        help="specify the width and height for the RGBD & mask images",
    )
    parser.add_argument(
        "--camera_fov",
        default=camera_fov,
        metavar="VALUE",
        help="specify the camera fov",
    )

    # Parameters for the motion
    parser.add_argument(
        "--motion_type",
        default=motion_type,
        choices=["rotation", "translation"],
        metavar="VALUE",
        help="specify the motion type",
    )
    parser.add_argument(
        "--motion_origin",
        nargs=3,
        default=motion_origin,
        metavar=("x", "y", "z"),
        type=float,
        help="specify the motion origin",
    )
    parser.add_argument(
        "--motion_axis",
        nargs=3,
        default=motion_axis,
        metavar=("x", "y", "z"),
        type=float,
        help="specify the motion axis",
    )

    return parser


if __name__ == "__main__":
    # Parse the parameters to get the image path and predicted motion
    args = get_parser().parse_args()

    # Get the attribute value from the parser
    rgb_path = args.rgb_path
    depth_path = args.depth_path
    mask_path = args.mask_path
    # Read camera intrinsics
    img_width = args.img_wh[0]
    img_height = args.img_wh[1]
    FOV = args.camera_fov
    # Read motion parameters
    motion_type = args.motion_type
    motion_origin = np.array(args.motion_origin)
    motion_axis = np.array(args.motion_axis)
    # normalize the motion axis
    motion_axis /= norm(motion_axis)

    # The width and height are the same
    fy, fx = getFocalLength(FOV / 180 * np.pi, img_height, img_width)
    cy = img_height / 2
    cx = img_width / 2
    # get the pc without background
    pcd_static, pcd_moving = get_pcd_from_rgbd_mask(
        rgb_path, depth_path, mask_path, fx, fy, cx, cy, 1000
    )

    # # Whether to convert to world coordinate
    # pcd_static.transform(extrinsic)
    # pcd_moving.transform(extrinsic)
    # axis_point = motion_origin + motion_axis
    # motion_origin = np.dot(extrinsic, np.array(list(motion_origin)+[1]))[:3]
    # axis_point = np.dot(extrinsic, np.array(list(axis_point)+[1]))[:3]
    # motion_axis = axis_point - motion_origin

    static_obb = pcd_static.get_oriented_bounding_box()
    total_obb = (pcd_static + pcd_moving).get_oriented_bounding_box()

    static_points = np.array(pcd_static.points)
    static_min = list(np.min(static_points, axis=0))
    static_max = list(np.max(static_points, axis=0))
    static_bbx = BBX(min_bound=static_min, max_bound=static_max, color=[1, 0, 0])

    total_points  = np.array((pcd_static + pcd_moving).points)
    total_min = list(np.min(total_points, axis=0))
    total_max = list(np.max(total_points, axis=0))
    total_bbx = BBX(min_bound=total_min, max_bound=total_max, color=[0, 0, 1])
    
    # # Visualize the camera coordinate
    # transformation = np.eye(4)
    # camera = getCamera(transformation, fx, fy, cx, cy)

    # Visualize the axis for the rotation or translation
    axis_point = motion_origin + motion_axis
    arrow = get_arrow(
        origin=motion_origin - motion_axis, end=axis_point, color=[0, 1, 1]
    )

    # Construct the small transformation for the animation
    if motion_type == "rotation":
        # rot_unit is for little rotation each frame
        rot_unit = 0.05
        # Construct the small rotation
        trans1 = np.eye(4)
        trans1[0:3, 3] = -motion_origin
        # Get the rotation matrix based on rotating around a axis start from origin point
        rot = axisAngleToRotationMatrix(motion_axis, rot_unit)
        trans2 = np.eye(4)
        trans2[0:3, 3] = motion_origin
        little_rotation = np.matmul(trans2, np.matmul(rot, trans1))

        # rot_unit is for little rotation each frame
        rot_unit = -0.05
        # Construct the small rotation
        trans1 = np.eye(4)
        trans1[0:3, 3] = -motion_origin
        # Get the rotation matrix based on rotating around a axis start from origin point
        rot = axisAngleToRotationMatrix(motion_axis, rot_unit)
        trans2 = np.eye(4)
        trans2[0:3, 3] = motion_origin
        neg_little_rotation = np.matmul(trans2, np.matmul(rot, trans1))
        
    elif motion_type == "translation":
        # trans_unit is for little translation each frame
        trans_unit = 0.1
        # Construct the small translation
        little_translation = [np.eye(4), np.eye(4)]
        little_translation[0][0:3, 3] += motion_axis * trans_unit
        little_translation[1][0:3, 3] -= motion_axis * trans_unit

    # Visualize the geometry
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd_static)
    vis.add_geometry(pcd_moving)
    vis.add_geometry(arrow)
    # vis.add_geometry(static_bbx.getMesh())
    # vis.add_geometry(total_bbx.getMesh())
    vis.add_geometry(static_obb)
    vis.add_geometry(total_obb)
    # static_stuff = [pcd_static, pcd_moving]
    flag = False
    #todo: add a point to judge the initial state (if it's close, don't do below things)
    index = 1
    last_min = np.array(total_min)
    last_max = np.array(total_max)
    last_neg_min = np.array(total_min)
    last_neg_max = np.array(total_max)
    min_scale = 10000
    for i in range(2000):
        if flag == True:
            if motion_type == "rotation":
                if index % 2 == 1: 
                    for j in range(index):
                        pcd_moving.transform(little_rotation)
                else:
                    for j in range(index):
                        pcd_moving.transform(neg_little_rotation)
                total_points  = np.array((pcd_static + pcd_moving).points)
                total_min = list(np.min(total_points, axis=0))
                total_max = list(np.max(total_points, axis=0))
                total_bbx.change(min_bound=total_min, max_bound=total_max)
                if index % 2 == 1:
                    scale = (np.array(total_max) - np.array(total_min)).prod()
                    min_scale = min(scale, min_scale)
                    # print(f"ZHENG: {(np.abs((np.array(total_min) - last_min)).sum() + np.abs((np.array(total_max) - last_max))).sum()}")
                    if (np.abs((np.array(total_min) - last_min)).sum() + np.abs((np.array(total_max) - last_max))).sum() <= 0.01 and scale < min_scale + 0.2:
                    # if (np.abs((np.array(total_min) - np.array(static_min))).sum() + np.abs((np.array(total_max) - np.array(static_max)))).sum() <= 0.1:
                        print(f"Find the close state {index*0.05/2}")
                        flag = False
                    last_min = np.array(total_min)
                    last_max = np.array(total_max)
                else:
                    scale = (np.array(total_max) - np.array(total_min)).prod()
                    min_scale = min(scale, min_scale)
                    # print(f"FAN: {((np.abs(np.array(total_min) - last_neg_min)).sum() + (np.abs(np.array(total_max) - last_neg_max))).sum()}")
                    if (np.abs((np.array(total_min) - last_min)).sum() + np.abs((np.array(total_max) - last_max))).sum() <= 0.01 and scale < min_scale + 0.2:
                    # if (np.abs((np.array(total_min) - np.array(static_min))).sum() + np.abs((np.array(total_max) - np.array(static_max)))).sum() <= 0.1:
                        print(f"Find the close state {index}")
                        flag = False
                    last_neg_min = np.array(total_min)
                    last_neg_max = np.array(total_max)
                index += 1
            elif motion_type == "translation":
                pcd_moving.transform(little_translation[int((i+10)/20)%2])
                
            vis.update_geometry(pcd_moving)
            vis.update_geometry(total_bbx.getMesh())
        vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()
