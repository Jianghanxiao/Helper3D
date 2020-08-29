from src import (
    URDFParser,
    URDFTree,
    SceneGraph,
)
import open3d as o3d

if __name__ == "__main__":
    URDF_file = "data/43074/mobility.urdf"
    # Parse the URDF file
    parser = URDFParser(URDF_file)
    parser.parse()
    # Construct the URDF tree
    links = parser.links
    joints = parser.joints
    tree = URDFTree(links, joints)
    # Construct the scene graph
    scene = SceneGraph(tree.root)
    mesh = scene.getMesh()
    print(mesh)
    o3d.visualization.draw_geometries([mesh])