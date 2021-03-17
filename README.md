# 3DHelper
This repo is used to integrate some useful code snippets for 3D visualization using open3D. 

## Current Problem
For open3d_URDF, the obj loading of open3D has some limitation on some texture materials. That part still need further use and test to control the URDF. Currently it can load the URDF objs correctly based on the parameters defined in the URDF

## open3d_RGBD
This project is used to visualize the RGBD image in point cloud form using open3D. It involves the transformation from image -> camera coordinate -> world coordinate. 

This part also provide some arrow helper to draw arrows in open3d.

Feel free to make your personal design for the intrinsic and extrinsic transformations.

## open3d_URDF
This project is used to load URDF and visualize it purely in python with open3D. It involves a simple scene graph implemetation, a simple (may not contain all attributes) URDF parser and URDF tree constructor.