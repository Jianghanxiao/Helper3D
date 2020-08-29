from .Transform import Transform
from .MeshNode import MeshNode
import numpy as np
from pytransform3d.rotations import (
    matrix_from_axis_angle,
    matrix_from_angle,
)


class SceneNode:
    def __init__(self, parent=None):
        self.parent = parent
        self.children = []
        # Store the local transform and world transform
        self.localTransform = Transform()
        if parent == None:
            self.worldMatrix = self.localTransform.getMatrix()
        else:
            self.worldMatrix = np.dot(
                self.parent.worldMatrix, self.localTransform.getMatrix()
            )
        # Store the mesh and deal with functions draw the mesh based on the transform
        self.meshNode = MeshNode()

    def setParent(self, parent):
        if parent == None:
            raise RuntimeError("Invalid Parent: parent is not in the SceneNode type")
        self.parent = parent
        self.worldMatrix = np.dot(
            self.parent.worldMatrix, self.localTransform.getMatrix()
        )

    def update(self):
        # Update the worldMatrix of current scene node
        self.worldMatrix = np.dot(
            self.parent.worldMatrix, self.localTransform.getMatrix()
        )

    def addChild(self, child):
        # child should also be SceneNode
        self.children.append(child)

    def addMesh(self, mesh):
        # mesh should be in open3d form
        self.meshNode.addMesh(mesh)

    def addMeshFile(self, mesh_file):
        self.meshNode.addMeshFile(mesh_file)

    def getMesh(self):
        # Get the new mesh based on the world Matrix (Assume that the matrix has been updatated)
        new_mesh = self.meshNode.getMesh(self.worldMatrix)
        return new_mesh

    def translate(self, translation):
        # translation should be in the array form np.arraay([float, float, float])
        transMat = np.array(
            [
                [1, 0, 0, translation[0]],
                [0, 1, 0, translation[1]],
                [0, 0, 1, translation[2]],
                [0, 0, 0, 1],
            ]
        )
        self.localTransform.translateMat(transMat)

    def rotate(self, axis, angle):
        # axis is in array form: [float, float, float], angle is in rad form
        array = np.zeros(4)
        array[0:3] = axis
        array[3] = angle
        # matrix here is 3*3
        matrix = matrix_from_axis_angle(array)
        rotMat = np.zeros(4, 4)
        rotMat[3, 3] = 1
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)

    def rotateX(self, angle):
        # matrix here is 3*3
        matrix = matrix_from_angle(0, angle)
        rotMat = np.zeros(4, 4)
        rotMat[3, 3] = 1
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)

    def rotateY(self, angle):
        # matrix here is 3*3
        matrix = matrix_from_angle(1, angle)
        rotMat = np.zeros(4, 4)
        rotMat[3, 3] = 1
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)

    def rotateZ(self, angle):
        # matrix here is 3*3
        matrix = matrix_from_angle(2, angle)
        rotMat = np.zeros(4, 4)
        rotMat[3, 3] = 1
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)
