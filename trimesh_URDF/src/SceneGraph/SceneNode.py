from .Transform import Transform
from .MeshNode import MeshNode
import numpy as np
import trimesh
import open3d as o3d


class SceneNode:
    def __init__(self, parent=None):
        self.parent = parent
        self.children = []
        self.name = None
        # Store the local transform and world transform
        self.localTransform = Transform()
        self.worldMatrix = np.eye(4)
        self.interactMatrix = np.eye(4)
        self._transformHelper = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # Store the mesh and deal with functions draw the mesh based on the transform
        self.meshNode = MeshNode()
        self.joint = None

    def getInfo(self):
        print(f"The joint info for {self.name}: {self.joint}\n")

    def setParent(self, parent):
        if parent == None:
            raise RuntimeError("Invalid Parent: parent is not in the SceneNode type")
        self.parent = parent
        self.worldMatrix = np.dot(
            self.parent.worldMatrix, self.localTransform.getMatrix()
        )

    def update(self):
        # Update the worldMatrix of current scene node
        if self.parent != None:
            self.worldMatrix = np.dot(
                self.parent.worldMatrix,
                np.dot(self.interactMatrix, self.localTransform.getMatrix()),
            )
        else:
            self.worldMatrix = np.dot(
                self.interactMatrix, self.localTransform.getMatrix()
            )
        # Update the worldMatrix for all it children
        for child in self.children:
            child.update()

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
        if new_mesh != None:
            new_mesh = [new_mesh]
        else:
            new_mesh = []
        # add mesh from all children
        for child in self.children:
            new_mesh += child.getMesh()
        new_mesh = trimesh.scene.scene.append_scenes(new_mesh)
        return new_mesh

    # Rotation is in degree, translation is in meter
    def interact(self, change):
        if self.joint.joint_type != "revolute" and self.joint.joint_type != "prismatic":
            print(
                f"The joint type {self.joint.type} cannot be articualted or not supported yet"
            )
            return
        if self.joint.joint_type == "revolute":
            self.translate(-self.joint.origin["xyz"])
            self.rotate(self.joint.axis, change)
            self.translate(self.joint.origin["xyz"])
        if self.joint.joint_type == "prismatic":
            self.translate(self.joint.axis * change)

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
        self.interactMatrix = np.dot(transMat, self.interactMatrix)

    def rotate(self, axis, angle):
        # Convert axis into 3*1 array
        axis = axis / np.linalg.norm(axis)
        axisAngle = axis * angle
        # matrix here is 3*3
        matrix = self._transformHelper.get_rotation_matrix_from_axis_angle(axisAngle)
        rotMat = np.eye(4)
        rotMat[0:3, 0:3] = matrix
        self.interactMatrix = np.dot(rotMat, self.interactMatrix)

    def translateLocal(self, translation):
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

    def rotateLocal(self, axis, angle):
        # Convert axis into 3*1 array
        axis = axis / np.linalg.norm(axis)
        axisAngle = axis * angle
        # matrix here is 3*3
        matrix = self._transformHelper.get_rotation_matrix_from_axis_angle(axisAngle)
        rotMat = np.eye(4)
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)

    def rotateXYZLocal(self, angle):
        # angle should be in array form [float, float, float] in radius
        matrix = self._transformHelper.get_rotation_matrix_from_xyz(angle)
        rotMat = np.eye(4)
        rotMat[0:3, 0:3] = matrix
        self.localTransform.rotateMat(rotMat)
