import math
import open3d as o3d
import numpy as np


def getFocalLength(FOV, height, width=None):
    # FOV is in radius, should be vertical angle
    if width == None:
        f = height / (2 * math.tan(FOV / 2))
        return f
    else:
        fx = height / (2 * math.tan(FOV / 2))
        fy = fx / height * width
        return (fx, fy)


def getCamera(transformation, fx, fy, cx, cy):
    # Return the camera and its corresponding frustum framework
    camera = o3d.geometry.TriangleMesh.create_coordinate_frame()
    camera.transform(transformation)
    # Add origin and four corner points in image plane
    points = []
    camera_origin = np.array([0, 0, 0, 1])
    points.append(np.dot(transformation, camera_origin)[0:3])
    # Calculate the four points for of the image plane
    magnitude = (cy**2 + cx**2 + fx**2) ** 0.5
    plane_points = [[-cy, -cx, -fx], [-cy, cx, -fx],
                    [cy, -cx, -fx], [cy, cx, -fx]]
    for point in plane_points:
        point = list(np.array(point) / magnitude)
        temp_point = np.array(point + [1])
        points.append(np.dot(transformation, temp_point)[0:3])
    # Draw the camera framework
    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
        [1, 2],
        [2, 4],
        [1, 3],
        [3, 4]
    ]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )

    return [camera, line_set]
