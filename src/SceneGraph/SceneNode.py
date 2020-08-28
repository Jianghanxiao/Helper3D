from .Transform import Transform
from .MeshNode import MeshNode
import numpy as np

class SceneNode:
    def __init__(self, parent=None):
        self.parent = parent
        self.children = []
        # Store the local transform and world transform
        self.localTransform = Transform()
        if parent == None:
            self.worldMatrix = self.localTransform.getMatrix()
        else:
            self.worldMatrix = np.dot(self.parent.worldMatrix, self.localTransform.getMatrix())    
        # Store the mesh and deal with functions draw the mesh based on the transform
        self.meshNode = MeshNode()

    def setParent(self, parent):
        if parent == None:
            raise RuntimeError('Invalid Parent: parent is not in the SceneNode type')
        self.parent = parent
        self.worldMatrix = np.dot(self.parent.worldMatrix, self.localTransform.getMatrix()) 

    def update(self):
        # Update the worldMatrix of current scene node
        self.worldMatrix = np.dot(self.parent.worldMatrix, self.localTransform.getMatrix())    
    
    def addChild(self, child):
        # child should also be SceneNode
        self.children.append(child)

    def addMesh(self, mesh):
        # mesh should be in open3d form
        self.meshNode.addMesh(mesh)
    
    def addMeshFile(self, mesh_file):
        self.meshNode.addMeshFile(mesh_file)