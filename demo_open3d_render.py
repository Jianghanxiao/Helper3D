import open3d as o3d
import numpy as np
from open3d_RGBD import getOpen3DFromTrimeshScene
from trimesh_URDF import getURDF
from trimesh_render import lookAt

MODELID = "7265"

URDFPATH = f"/local-scratch/localhome/hja40/Desktop/Dataset/data/models3d/partnetsim/mobility_v1_alpha5/{MODELID}/mobility.urdf"

IMGSIZE = 512

if __name__ == "__main__":
    urdf, controller = getURDF(URDFPATH)
    # Interact with the URDF
    # controller["link_0"].interact(-0.5235987755982988)
    trimesh_scene = urdf.getMesh()

    mesh = getOpen3DFromTrimeshScene(trimesh_scene)

    # Use visualizer to render the object
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=IMGSIZE, height=IMGSIZE)
    vis.add_geometry(mesh)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame())
    view_control = vis.get_view_control()
    
    # Retrieve the camera parameters
    camera_params = view_control.convert_to_pinhole_camera_parameters()
    # Set the intrinsic parameters
    camera_params.intrinsic.width = IMGSIZE
    camera_params.intrinsic.height = IMGSIZE
    intrinsic = np.eye(3)
    intrinsic[0, 0] = IMGSIZE / 2
    intrinsic[1, 1] = IMGSIZE / 2
    intrinsic[0, 2] = IMGSIZE / 2 - 0.5
    intrinsic[1, 2] = IMGSIZE / 2 - 0.5
    camera_params.intrinsic.intrinsic_matrix = intrinsic
    # Set the extrinsic parameters, yz_flip is for Open3D camera configuration
    camera_pose = lookAt(eye=np.array([-2., 1., 1.]), target=np.array([0. ,0., 0.]), up=np.array([0.0, 0.0, 1.0]), yz_flip=True)
    camera_params.extrinsic = np.linalg.inv(camera_pose)
    # Set the camera parameters
    view_control.convert_from_pinhole_camera_parameters(camera_params)

    # The second way to control the camera, but not easy to use as our lookat
    # # view_control.set_front([-1, 0, 0])
    # # view_control.set_up([0, 0, 1])
    # # view_control.set_lookat([-20, 0, 0])

    # Update the visualizer
    vis.poll_events() 
    vis.capture_screen_image("test1.png")

    vis.destroy_window()