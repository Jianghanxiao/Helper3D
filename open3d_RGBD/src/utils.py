import numpy as np


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