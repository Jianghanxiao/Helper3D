 # Make use of the codebase from https://github.com/Jianghanxiao/3DHelper
from trimesh_render import lookAt, getSpherePositions
from trimesh_URDF import getURDF
import numpy as np
from PIL import Image
import pyrender
import io
import os
import json

MODELID = "7265"

URDFPATH = f"/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/{MODELID}/mobility.urdf"
OUTPUTDIR = "/local-scratch/localhome/hja40/Desktop/Research/proj-csopd/dataset"

IMGSIZE = 512

def existDir(dir):
    if not os.path.exists(dir):
        os.mkdir(dir)

if __name__ == "__main__":
    existDir(f"{OUTPUTDIR}")
    existDir(f"{OUTPUTDIR}/{MODELID}")

    urdf, controller = getURDF(URDFPATH)

    # Interact with the URDF
    # controller["link_0"].interact(-0.5235987755982988)

    trimesh_scene = urdf.getMesh()

    # Load the trimesh scene into pyrender and set the background transparent
    scene = pyrender.Scene.from_trimesh_scene(trimesh_scene)
    scene.ambient_light = np.ones(3) * 0.3
    scene.bg_color = np.array([0.0, 0.0, 0.0, 0.0])

    # Add the camera
    camera = pyrender.PerspectiveCamera(yfov=np.pi / 2.0, aspectRatio=1.0)
    camera_node = scene.add(camera)
    # Add the light
    light = pyrender.DirectionalLight(
        color=np.ones(3),
        intensity=5.0,
    )
    light_node = scene.add(light)

    # Get the positions of the cameras on a sphere centered at the object center
    center = trimesh_scene.centroid
    positions = getSpherePositions(center=center, radius=2)
    # Loop over the camera positions and render the images
    for i, pos in enumerate(positions):
        # Set the camera view and render the image
        camera_pose = lookAt(eye=pos, target=center, up=np.array([0.0, 0.0, 1.0]))
        scene.set_pose(camera_node, camera_pose)
        scene.set_pose(light_node, camera_pose)

        r = pyrender.OffscreenRenderer(IMGSIZE, IMGSIZE)
        rgb, depth = r.render(scene, flags=pyrender.RenderFlags.RGBA)
        r.delete()

        # Save the rgb image
        rgb = Image.fromarray(rgb, mode='RGBA')
        existDir(f"{OUTPUTDIR}/{MODELID}/rgb")
        rgb.save(f"{OUTPUTDIR}/{MODELID}/rgb/{i}.png")

        # Save the depth image
        existDir(f"{OUTPUTDIR}/{MODELID}/depth")
        np.save(f"{OUTPUTDIR}/{MODELID}/depth/{i}.npy", depth)

        # Save the annotation of camera intrinsic and extrinsic
        # This intrinsic is for fov=90, aspectRatio=1.0
        intrinsic = np.eye(4)
        intrinsic[0, 0] = IMGSIZE / 2
        intrinsic[1, 1] = IMGSIZE / 2
        intrinsic[0, 2] = IMGSIZE / 2
        intrinsic[1, 2] = IMGSIZE / 2
        # Get the extrisic matrix
        extrinsic = camera_pose
       
        anno = {"intrinsic": intrinsic.tolist(), "extrinsic": extrinsic.tolist()}
        existDir(f"{OUTPUTDIR}/{MODELID}/anno")
        with open(f"{OUTPUTDIR}/{MODELID}/anno/{i}.json", "w") as f:
            json.dump(anno, f)
