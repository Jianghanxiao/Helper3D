# This script is used to validate the image rendered is correct
from Helper3D import (
    getURDF,
    getOpen3DFromTrimeshScene,
    getPcdFromRgbd,
    getCamera
)
import open3d as o3d
import json
import numpy as np

MODELID = "7265"
URDFPATH = f"/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/{MODELID}/mobility.urdf"
DATAPATH = f"/local-scratch/localhome/hja40/Desktop/Research/proj-csopd/dataset/{MODELID}"
INDEX = "0"

if __name__ == "__main__":
    # Load the URDF and get the trimesh of the target state of the model
    urdf, controller = getURDF(URDFPATH)
    trimesh_scene = urdf.getMesh()
    mesh = trimesh_scene.geometry["material_1_1"]

    # Use open3D to visualize the mesh (the texture cannot be loaded into open3d)
    # Just to verify the RGBD and camera information is correct
    whole_object = getOpen3DFromTrimeshScene(trimesh_scene, random_color=False)
    world = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # o3d.visualization.draw_geometries([whole_object, world])

    # Load the RGBD and corresponding camera information
    rgb = np.array(o3d.io.read_image(f'{DATAPATH}/rgb/{INDEX}.png'), dtype=np.float64)
    depth_img = np.load(f'{DATAPATH}/depth/{INDEX}.npy') 
    anno_file = open(f'{DATAPATH}/anno/{INDEX}.json')
    anno = json.load(anno_file)
    intrinsic = np.array(anno["intrinsic"])
    extrinsic = np.array(anno["extrinsic"])

    pcd = getPcdFromRgbd(rgb, depth_img, intrinsic=intrinsic, alpha_filter=True)
    pcd.transform(extrinsic)

    world = o3d.geometry.TriangleMesh.create_coordinate_frame()

    # Visualize the camera
    fx = intrinsic[0, 0]
    fy = intrinsic[1, 1]
    cx = intrinsic[0, 2]
    cy = intrinsic[1, 2]
    camera = getCamera(extrinsic, fx, fy, cx, cy)

    o3d.visualization.draw_geometries([pcd, whole_object, world] + camera)