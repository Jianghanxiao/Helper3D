import numpy as np

# lookAt function implementation
def lookAt(eye, target, up):
    # Normalize the up vector
    up /= np.linalg.norm(up)
    forward = eye - target
    forward /= np.linalg.norm(forward)
    if np.dot(forward, up) == 1 or np.dot(forward, up) == -1:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    new_up = np.cross(right, forward)
    new_up /= np.linalg.norm(new_up)

    # Construct a rotation matrix from the right, new_up, and -forward vectors
    rotation = np.eye(4)
    rotation[:3, :3] = np.row_stack((right, new_up, forward))

    # Apply a translation to the camera position
    translation = np.eye(4)
    translation[:3, 3] = [
        np.dot(right, eye),
        np.dot(new_up, eye),
        -np.dot(forward, eye),
    ]

    camera_pose = np.linalg.inv(np.matmul(translation, rotation))

    # The above transformation is left-handed, but trimesh or pyrender uses right-handed
    camera_pose[1, :] *= -1

    return camera_pose
