from src import (
    URDFParser,
    URDFTree,
    SceneGraph,
)
import open3d as o3d

if __name__ == "__main__":
    URDF_file = "/Users/shawn/Desktop/3DHelper/data/45194_urdf/mobility.urdf"
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
    # mesh.paint_uniform_color([0.5, 0.5, 0.5])
    print(mesh)
    o3d.visualization.draw_geometries(mesh)