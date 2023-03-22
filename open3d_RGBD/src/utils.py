import numpy as np
from .model import (
    getArrow,
    BBX,
)
import open3d as o3d
from .rotation_utils import eulerAnglesToRotationMatrix

# Convert the trimesh scene into open3d geometry
def getOpen3DFromTrimeshScene(trimesh_scene, random_color=True):
    mesh = o3d.geometry.TriangleMesh()
    geo_trans_mapping = {}
    # Get the mapping between the geometry key and its corresponding transformation
    for key in trimesh_scene.graph.nodes_geometry:
        geo_trans_mapping[trimesh_scene.graph[key][1]] = trimesh_scene.graph[key][0]

    for key, geometry in trimesh_scene.geometry.items():
        # Take the scene transformation into account
        geometry.apply_transform(np.dot(trimesh_scene.graph["world"][0], geo_trans_mapping[key]))
        # Convert the mesh into open3d
        temp_mesh = geometry.as_open3d
        if random_color:
            temp_mesh.paint_uniform_color(np.random.rand(3))
        mesh += temp_mesh
    return mesh   


def getConventionTransform(source):
    transformation = np.matrix(np.eye(4))
    if(source == 'partnetsim' or source == 'sapien' or source == 'SAPIEN'):
        transformation[0, 0] = -1
        transformation[2, 2] = -1
    elif(source == 'shape2motion'):
        transformation[0:3, 0:3] = np.matrix([[0,  0, -1],
                                              [-1,  0,  0],
                                              [0, 1,  0]])
    return transformation.I


def getMotion(motion, transformation, state='current', is_real=False):
    # Used for latest verion: initial/current origin, axis (In camera coordinate)
    # State can be 'current' or 'initial'
    # # Visualize 3D BBX
    # bbx_name = f'{state}_3dbbx'
    # bbx_points = motion[bbx_name]['points']
    # bbx_lines = motion[bbx_name]['lines']
    # bbx = BBX(points=bbx_points, lines=bbx_lines, color=[0.8, 0.2, 0])
    # bbx = bbx.getMesh()
    if not is_real:
        ''' Visualize 3D BBX with the part pose '''
        part_dimension = np.array(motion['partPose']['dimension'])
        part_translation = np.array(motion['partPose']['translation'])
        part_rotation = np.array(motion['partPose']['rotation'])
        # Construct the intial consistent bbx in camera coordinate
        min_bound = (-part_dimension[0]/2,
                    -part_dimension[1]/2, -part_dimension[2]/2)
        max_bound = (part_dimension[0]/2, part_dimension[1]/2, part_dimension[2]/2)
        bbx = BBX(min_bound=min_bound, max_bound=max_bound)
        # Construct the transformation matrix
        pose_transformation = np.eye(4)
        pose_transformation[0:3, 3] = part_translation
        pose_transformation[0:3, 0:3] = eulerAnglesToRotationMatrix(part_rotation)
        bbx.transform(np.dot(transformation, pose_transformation))
        bbx = bbx.getMesh()

    # import pdb
    # pdb.set_trace()

    ''' Visualize motion axis (still need consider translation visualization) '''
    origin_name = f'{state}_origin'
    origin = np.dot(transformation, np.array(motion[origin_name] + [1]))[0:3]
    axis_name = f'{state}_axis'
    axis_point = list(
        np.array(motion[origin_name]) + np.array(motion[axis_name]))
    axis_point = np.dot(transformation, np.array(axis_point + [1]))[0:3]

    arrow = getArrow(origin=origin, end=axis_point, color=[0, 1, 1])
    # import pdb
    # pdb.set_trace()
    if not is_real:
        return [bbx, arrow]
    else:
        return [arrow]


# def getMotion(motion):
#     # Used for the old-version data which contains intial_pose and current_pose
#     current_pose = np.matrix(np.reshape(motion['current_pose'], (4, 4)).T)
#     # Visualize 3D BBX
#     min_bound_raw = motion['3dbbx']['min']
#     max_bound_raw = motion['3dbbx']['max']
#     min_bound = np.array(
#         [min_bound_raw['x'], min_bound_raw['y'], min_bound_raw['z']])
#     max_bound = np.array(
#         [max_bound_raw['x'], max_bound_raw['y'], max_bound_raw['z']])
#     bbx = BBX(min_bound=min_bound, max_bound=max_bound, color=[0.8, 0.2, 0])
#     bbx.transform(current_pose)
#     bbx = bbx.getMesh()
#     # Visualize motion axis (still need consider translation visualization)
#     origin_raw = motion['origin']
#     origin = np.array([origin_raw['x'], origin_raw['y'], origin_raw['z']])
#     axis_raw = motion['axis']
#     axis = np.array([axis_raw['x'], axis_raw['y'], axis_raw['z']])
#     arrow = get_arrow(origin=origin-axis, vec=3*axis, color=[0, 1, 1])
#     arrow.transform(current_pose)

#     return [bbx, arrow]
