from trimesh_URDF import getURDF
import numpy as np

# The model is from SAPIEN dataset (https://sapien.ucsd.edu/)
URDFPATH = "/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/45194/mobility.urdf"


if __name__ == '__main__':
    urdf, controller = getURDF(URDFPATH, JointInfo=True)


    # Interact with the URDF
    controller["link_0"].interact(np.pi/2)
    controller["link_1"].interact(np.pi/4)
    controller["link_2"].interact(0.2)

    # Get the final visualziation
    mesh = urdf.getMesh()

    import trimesh
    coordinate = trimesh.creation.axis()
    mesh.add_geometry(coordinate)


    mesh.show()
