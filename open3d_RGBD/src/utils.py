import numpy as np
from .model import (
    get_arrow,
    BBX,
)
import open3d as o3d


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


def getMotion(motion, state='current'):
    # Used for latest verion: initial/current origin, axis and 3dbbx
    # State can be 'current' or 'initial'
    # Visualize 3D BBX
    bbx_name = f'{state}_3dbbx'
    bbx_points = motion[bbx_name]['points']
    bbx_lines = motion[bbx_name]['lines']
    bbx = BBX(points=bbx_points, lines=bbx_lines, color=[0.8, 0.2, 0])
    bbx = bbx.getMesh()
    # Visualize motion axis (still need consider translation visualization)
    origin_name = f'{state}_origin'
    origin = np.array(motion[origin_name])
    axis_name = f'{state}_axis'
    axis = np.array(motion[axis_name])
    arrow = get_arrow(origin=origin, end=origin+axis, color=[0, 1, 1])
    
    return [bbx, arrow]


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
