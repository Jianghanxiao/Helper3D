import numpy as np
import open3d as o3d

def getPcdFromRgbd(
    rgb,
    depth,
    fx=None,
    fy=None,
    cx=None,
    cy=None,
    intrinsic=None,
    depth_scale=1,
    alpha_filter=False,
    is_opengl=True
):
    # Make the rgb go to 0-1
    if rgb.max() > 1:
        rgb /= 255.0
    # Convert the unit to meter
    depth /= depth_scale

    if intrinsic is None:
        intrinsic = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])

    H, W = depth.shape
    x, y = np.meshgrid(np.arange(W), np.arange(H))
    x = x.reshape(-1)
    y = y.reshape(-1)
    depth = depth.reshape(-1)
    points = np.stack([x, y, np.ones_like(x)], axis=1)
    points = points * depth[:, None]

    intrinsic = intrinsic[:3, :3]
    points = points @ np.linalg.inv(intrinsic).T

    if is_opengl:
        # Convert from opencv convention to opengl convention
        points[:, 1] *= -1 
        points[:, 2] *= -1

    if alpha_filter:
        mask = rgb[:, :, 3] == 1
        mask = mask.reshape(-1)
    rgb = rgb[:, :, :3].reshape(-1, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[mask])
    pcd.colors = o3d.utility.Vector3dVector(rgb[mask])

    return pcd