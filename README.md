# Helper3D
This repo is used to integrate some useful code snippets for 3D visualization using Open3D and URDF supporter using trimesh (open3d texture support is not good....). 

<table>
  <tr>
    <td><img src="images/ex.png" alt="image1"></td>
    <td><img src="images/ex1.png" alt="image2"></td>
    <td><img src="images/ex2.png" alt="image3"></td>
    <td><img src="images/ex3.png" alt="image3"></td>
  </tr>
</table>


## Latest Update
22.4.1 Get joint information in the world coordinate; Fix the lookAt function bug; Support motion update and improve the update function to lazy update to accelerate
22.3.15? Trimesh URDF -> Support URDF into trimesh.Scene format; Support control the URDF with simple code (see example.py in trimesh_URDF)

## open3d_RGBD
!! This project will be cleaned and restructured to have better codebase. Currently just use it as snippet
This project is used to visualize the RGBD image in point cloud form using open3D. It involves the transformation from image -> camera coordinate -> world coordinate. 

This part also provide some arrow helper to draw arrows in open3d.

Feel free to make your personal design for the intrinsic and extrinsic transformations.

## trimesh_URDF
This project is used to load URDF, visualize and intereact with it using simple code with Trimesh. It inolves a simple scene graph implemetation, a simple (may not contain all attributes) URDF parser and URDF tree constructor. (Check the `demo_RRDF.py` code to see how to play with it).

## trimesh_render
This codebase is just for easy rendering RGB with trimesh (But TBH, Pyrender is much easier to use and has more advanced functions)