# Helper3D
This repo is used to integrate some useful code snippets for 3D visualization using Open3D and URDF supporter using trimesh (open3d texture support is not good....). 

<table>
  <tr>
    <td><img src="images/ex.png" alt="image1"></td>
    <td><img src="images/ex1.png" alt="image2"></td>
    <td><img src="images/ex2.png" alt="image3"></td>
  </tr>
</table>


## Latest Update
Trimesh URDF -> Support URDF into trimesh.Scene format; Support control the URDF with simple code (see example.py in trimesh_URDF)

## open3d_RGBD
This project is used to visualize the RGBD image in point cloud form using open3D. It involves the transformation from image -> camera coordinate -> world coordinate. 

This part also provide some arrow helper to draw arrows in open3d.

Feel free to make your personal design for the intrinsic and extrinsic transformations.

## trimesh_URDF
This project is used to load URDF, visualize and intereact with it using simple code with Trimesh. It inolves a simple scene graph implemetation, a simple (may not contain all attributes) URDF parser and URDF tree constructor. (Check the example code to see how to play with it).

