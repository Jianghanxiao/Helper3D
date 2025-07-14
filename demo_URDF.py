 # Make use of the codebase from https://github.com/Jianghanxiao/3DHelper
from trimesh_render import lookAt, getSpherePositions
from trimesh_URDF import getURDF
import numpy as np
from PIL import Image
import trimesh
import io
import os

MODELID = "13928"

URDFPATH = f"/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/{MODELID}/mobility.urdf"


def getRGB(scene):
    data = scene.save_image()
    rgb = Image.open(io.BytesIO(data))
    return rgb


if __name__ == "__main__":
    urdf, controller = getURDF(URDFPATH, JointInfo=True)


    # Interact with the URDF
    controller["link_4"].interact(np.pi/2)
    # mesh = urdf.getMesh()
    controller["link_1"].interact(np.pi/2)
    # mesh = urdf.getMesh()
    controller["link_0"].interact(np.pi/2)
    # mesh = urdf.getMesh()
    controller["link_3"].interact(np.pi/2)
    mesh = urdf.getMesh()

    # Get the motion parameters in the world coordinate
    urdf.updateMotionWorld()
    for name, node in controller.items():
        if node.joint is not None:
            print(f"The joint in the world coordinate for {name}: origin {node.origin_world}, axis {node.axis_world}")
    
    coordinate = trimesh.creation.axis(origin_size=0.1)

    mesh.show()
