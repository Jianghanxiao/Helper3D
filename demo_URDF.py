 # Make use of the codebase from https://github.com/Jianghanxiao/3DHelper
from trimesh_render import lookAt, getSpherePositions
from trimesh_URDF import getURDF
import numpy as np
from PIL import Image
import trimesh
import io
import os

MODELID = "7265"

URDFPATH = f"/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/{MODELID}/mobility.urdf"
OUTPUTDIR = "/local-scratch/localhome/hja40/Desktop/Research/proj-csopd/dataset"

if not os.path.exists(OUTPUTDIR):
    os.mkdir(OUTPUTDIR)


def getRGB(scene):
    data = scene.save_image()
    rgb = Image.open(io.BytesIO(data))
    return rgb


if __name__ == "__main__":
    urdf, controller = getURDF(URDFPATH)

    # Interact with the URDF
    controller["link_0"].interact(-0.5235987755982988)

    mesh = urdf.getMesh()

    coordinate = trimesh.creation.axis(origin_size=0.1)
    mesh.add_geometry(coordinate)

    mesh.show()
