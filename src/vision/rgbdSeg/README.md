# Plane Segmentation using RGB-D Cameras

A few different approaches were investigated for surface segmentation (and drivable surface segmentation in particular). An example of a very similar application of an autonomous mobile robot with an RGB-D camera is [An Open-Source Low-Cost Mobile Robot System With an RGB-D Camera and Efficient Real-Time Navigation Algorithm](https://ieeexplore.ieee.org/abstract/document/9970319). The authors implement their own algorithm for drivable surface segmentation, but they reference another paper [Real-Time Plane Segmentation using RGB-D Cameras](https://link.springer.com/chapter/10.1007/978-3-642-32060-6_26). First, surfaces are segmented, and with the computed normals, we find the most probable ground surface based on the normal angle. We will subsequently need to compute the distance to obstacles in the point cloud and determine if the robot may proceed in a certain direction before striking an object.

## [RGBDPlaneDetection](https://github.com/chaowang15/RGBDPlaneDetection)

The best repository for RGBD plane detection I could find is [RGBDPlaneDetection](https://github.com/chaowang15/RGBDPlaneDetection). This provides the point cloud plane fitting and plane refinement. In order to adapt this code, I copied over the `plane_detection.cpp` and `include/` made the necessary adjustments to accept the realsense images and then built the project. The results are much better than any CNN I could find. See the results below:


![Sample segmentation](sample_segmentation.png "Sample segmentation using realsense camera")