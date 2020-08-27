import numpy as np


class Link:
    def __init__(self, link_name):
        self.link_name = link_name
        # Naming rule: concaten tag name as the variable name, and attribute name as the key
        self.has_visual = False
        self.visual_origin = {"xyz": np.array([0, 0, 0]), "rpy": np.array([0, 0, 0])}
        self.visual_geometry_mesh = {"filename": None}

    def setVisualOriginXyz(self, xyz):
        self.has_visual = True
        self.visual_origin["xyz"] = np.array(xyz)

    def setVisualOriginRpy(self, rpy):
        self.has_visual = True
        self.visual_origin["rpy"] = np.array(rpy)

    def setVisualGeometryMeshFilename(self, filename):
        self.has_visual = True
        self.visual_geometry_mesh["filename"] = filename

    def __repr__(self):
        output = {}
        output["name"] = self.link_name
        output["has_visual"] = self.has_visual
        output["origin"] = self.visual_origin
        output["filename"] = self.visual_geometry_mesh["filename"]

        return str(output)
