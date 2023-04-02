from .src import (
    URDFParser,
    URDFTree,
    SceneGraph,
)

# Load the URDF into trimesh, return the urdf and controller
# JointInfo: whether to print all the joint information
def getURDF(path, JointInfo=False):
    # Initialize the URDF
    # Parse the URDF file
    parser = URDFParser(path)
    parser.parse()
    # Construct the URDF tree
    links = parser.links
    joints = parser.joints
    tree = URDFTree(links, joints)
    # Construct the scene graph
    urdf = SceneGraph(tree.root)

    # Get the controller
    # Get all the nodes
    controller = urdf.getNodes()
    if JointInfo:
        urdf.printInfo()

    return urdf, controller
