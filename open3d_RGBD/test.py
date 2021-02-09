import open3d as o3d
import numpy as np
points = [
        [0, 0, 0],
        [10, 0, 0],
        [0, 10, 0],
        [0, 0, 10]
]
lines = [
        [0, 1],
        [0, 2],
        [0, 3]
]
line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(points),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(points[1:])

camera = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 1], size=0.5)
# camera.translate([0, 0, 1], True)
# print(np.array(line_set.points))
# camera1 = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 1, 0], size=0.5)
# camera2 = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[1, 0, 0], size=0.5)

bbx = o3d.geometry.AxisAlignedBoundingBox([0,0,0], [1,1,1])

o3d.visualization.draw_geometries([line_set, bbx, camera])