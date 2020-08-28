from .Transform import Transform
import numpy as np

class SceneNode:
    def __init__(self):
         self.parent = None
         self.children = []
         # Store the local transform and world transform
         self.localTransform = Transform()
         self.worldMatrix = np.identity(4)
         # Store the mesh and deal with functions draw the mesh based on the transform
         self.meshNode = None