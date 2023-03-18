from src import (
    URDFParser,
    URDFTree,
    SceneGraph,
)
import numpy as np

# The model is from SAPIEN dataset (https://sapien.ucsd.edu/)
URDFPATH = "/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/45194/mobility.urdf"


if __name__ == '__main__':
    # Initialize the URDF
    # Parse the URDF file
    parser = URDFParser(URDFPATH)
    parser.parse()
    # Construct the URDF tree
    links = parser.links
    joints = parser.joints
    tree = URDFTree(links, joints)
    # Construct the scene graph
    scene = SceneGraph(tree.root)

    # Get the controller
    # Get all the nodes
    controller = scene.getNodes()
    # Print the joint information for all the nodes
    scene.getInfo()


    # Interact with the URDF
    controller["link_0"].interact(np.pi/2)
    controller["link_1"].interact(np.pi/4)
    controller["link_2"].interact(0.2)

    # Get the final visualziation
    mesh = scene.getMesh()

    import trimesh
    coordinate = trimesh.creation.axis()
    mesh.add_geometry(coordinate)


    mesh.show()
