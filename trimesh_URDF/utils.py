import copy
import numpy as np
import trimesh
from .src import (
    URDFParser,
    URDFTree,
    SceneGraph,
)

# Sample Points from trimesh.Scene, different point number for different geometry based on the area ratio. The number of points is not strict, just oversample
def SampleSurfaceFromTrimeshScene(trimesh_scene, num_points):
    geo_trans_mapping = {}
    # Get the mapping between the geometry key and its corresponding transformation
    for key in trimesh_scene.graph.nodes_geometry:
        geo_trans_mapping[trimesh_scene.graph[key][1]] = trimesh_scene.graph[key][0]

    points = []
    colors = []
    normals = []
    # Calculate the are for the mesh with uv
    total_area = 0
    for key, geometry in trimesh_scene.geometry.items():
        if geometry.visual.uv is not None:
            total_area += geometry.area

    # Get the points from the geometries in the trimesh.Scene
    for key, geometry in trimesh_scene.geometry.items():
        # Need to deepcopy, or it will influence the original mesh
        geometry = copy.deepcopy(geometry)
        # Take the scene transformation into account
        geometry.apply_transform(np.dot(trimesh_scene.graph["world"][0], geo_trans_mapping[key]))
        # Check the number of points based on the area ratio of the whole trimesh scene
        num_geo_points = int(geometry.area / total_area * num_points)
        # Some geometry may not have texture uv
        if geometry.visual.uv is None:
            continue
        else:
            result = trimesh.sample.sample_surface(geometry, num_geo_points, sample_color=True)
            colors.append(np.array(result[2])[:, :3] / 255)
        points.append(np.array(result[0]))
        normals.append(geometry.face_normals[result[1]])
    
    # Concatenate the array
    points = np.concatenate(points, axis=0)
    colors = np.concatenate(colors, axis=0)
    normals = np.concatenate(normals, axis=0)
    
    import pdb
    pdb.set_trace()

    return points, colors, normals

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
