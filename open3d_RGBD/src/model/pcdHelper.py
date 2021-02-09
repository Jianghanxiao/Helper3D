import numpy as np
import open3d as o3d


def get_pcd_from_rgbd(color_img_path, depth_img_path, fx, fy, cx, cy, background_filter=None):
    # Self-designed with depth unit transform and background filter
    # Calculate the projection matrix by hand; Add manual background filter
    color_raw = np.array(o3d.io.read_image(color_img_path)) / 255
    # Convert the unit to meter
    depth_raw = np.array(o3d.io.read_image(depth_img_path)) / 1000
    height, width = np.shape(depth_raw)
    points = []
    colors = []

    for y in range(height):
        for x in range(width):
            if depth_raw[y][x] < 1:
                continue
            colors.append(color_raw[y][x])
            points.append([(x - cx) * (depth_raw[y][x] / fx),
                           -(y - cy) * (depth_raw[y][x] / fy), -depth_raw[y][x]])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors)[:, 0:3])

    return pcd


def get_pcd_from_whole_rgbd(color_img_path, depth_img_path, fx, fy, cx, cy,):
    # Use the open3d function to visualize the whole pc
    # Read the color and depth image
    color_raw = o3d.io.read_image(color_img_path)
    depth_raw = o3d.io.read_image(depth_img_path)
    height, width = np.shape(np.array(depth_raw))
    # Get the colored RGBD
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw, depth_raw, convert_rgb_to_intensity=False)
    # Get the point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy))

    return pcd
