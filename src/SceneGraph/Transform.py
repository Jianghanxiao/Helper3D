import numpy as np

# Borrow idea form pySG repo: https://github.com/jmanek/pySG
class Transform:
    # Only support matrix currently
    def __init__(self):
        self._matrix = np.eye(4)
        # Matrix for calculating the latest matrix, the order will always be sclae -> rotate -> translate
        self._transMat = np.eye(4)
        self._rotMat = np.eye(4)
        self._scaleMat = np.eye(4)
    
    def getMatrix(self):
        self.updateMatrix()
        return self._matrix

    def updateMatrix(self):
        self._matrix = np.dot(self._transMat, np.dot(self._rotMat, self._scaleMat))

    def translateMat(self, transMat):
        # Apply the translation after previous _transMat
        self._transMat = np.dot(transMat, self._transMat)

    def rotateMat(self, rotMat):
        # Apply the rotation after previous _rotMat
        self._rotMat = np.dot(rotMat, self._rotMat)

    def scaleMat(self, scaleMat):
        # Apply the scale after previous _scaleMat
        self._scaleMat = np.dot(scaleMat, self._scaleMat)