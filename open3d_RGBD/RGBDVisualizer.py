import open3d as o3d
import math

from src import (
    getFocalLength,
    get_pcd_from_rgbd,
    get_pcd_from_whole_rgbd
)

DATA_PATH = "/Users/apple/Desktop/3DHelper/data/7128/"

if __name__ == "__main__":
    color_img = f'{DATA_PATH}origin/7128-0-0.png'
    depth_img = f'{DATA_PATH}depth/7128-0-0_d.png'
    img_height = 256
    FOV = 50
    # The width and height are the same
    focal_length = getFocalLength(FOV / 180 * math.pi, img_height)
    # Display the pc with background
    # pcd = get_pcd_from_whole_rgbd(color_img, depth_img, focal_length, focal_length, 128, 128)

    pcd = get_pcd_from_rgbd(color_img, depth_img,
                            focal_length, focal_length, img_height / 2, img_height / 2, 1000)
    o3d.visualization.draw_geometries([pcd])
