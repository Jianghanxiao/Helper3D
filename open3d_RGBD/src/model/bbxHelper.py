import open3d as o3d
import numpy as np


class BBX:
    # todo: waiting for official bug fixing
    # Becasue offical oriented bounding box has rotate bug, write one for easily use
    def __init__(self, min_bound=None, max_bound=None, points=None, lines=None, color=[0, 0, 0]):
        if(points != None and lines != None):
            # Initializing the bounding box with points and lines
            self.line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            self.line_set.paint_uniform_color(color)
        elif(min_bound != None and max_bound != None):
            # This init function will only generate axis aligned bounding box
            # But it can be transformed with a transformation matrix
            x_min, y_min, z_min = min_bound
            x_max, y_max, z_max = max_bound
            # Create the bounding box
            points = []
            points.append([x_min, y_min, z_min])
            points.append([x_max, y_min, z_min])
            points.append([x_max, y_max, z_min])
            points.append([x_min, y_max, z_min])
            points.append([x_min, y_min, z_max])
            points.append([x_max, y_min, z_max])
            points.append([x_max, y_max, z_max])
            points.append([x_min, y_max, z_max])
            lines = [
                [0, 1],
                [1, 2],
                [2, 3],
                [3, 0],
                [4, 5],
                [5, 6],
                [6, 7],
                [7, 4],
                [0, 4],
                [1, 5],
                [2, 6],
                [3, 7]
            ]
            self.line_set = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector(points),
                lines=o3d.utility.Vector2iVector(lines),
            )
            self.line_set.paint_uniform_color(color)
        else:
            raise RuntimeError('Error when initializing the bounding box')

    def getMesh(self):
        return self.line_set

    def transform(self, transformation):
        self.line_set.transform(transformation)
