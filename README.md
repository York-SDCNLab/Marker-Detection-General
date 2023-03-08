# Background

The [Intensity Image-based LiDAR Fiducial Marker (IILFM)](https://github.com/York-SDCNLab/IILFM) can provide 3D artificial fiducials for light detection and ranging (LiDAR) sensors. LiDARs with a steering mirror have a limited Field of View (FoV) and it is required to wait for the increase of coverage in the FoV to obtain a dense point cloud. Please refer to the following Figures. Hence, if the adopted LiDAR has a steering mirror, the original IILFM system requires that the LiDAR remain stationary (placed on a tripod) since the system needs a dense single-view point cloud as the input. 
![fig0](https://user-images.githubusercontent.com/58899542/208346898-9169304a-56b9-47cb-b0a1-036913315471.png)<br>
(a) Examples of LiDARs with a steering mirror. <br>
(b) A schematic diagram of the LiDAR with a steering mirror.<br>
<img width="450" height="150" src="https://user-images.githubusercontent.com/58899542/208347303-2e4904b5-90d4-4fb4-94f2-969c9e310645.png"/> <br>
Take the Livox MID-40 as an example.<br>
(a) The LiDAR only has a front-facing and conical-shaped FoV spanning 38.4 degrees. <br>
(b) The point cloud under different integration times (0.1 s, 0.5 s, and 1 s). The FoV coverage increases over time due to the superimposition of frames.<br>
However, to scan a large scene, for instance, a campus, using a LiDAR that resembles Livox MID-40, the LiDAR has to be moved in the scene due to its small FoV. This requires that the 3D points are captured at multiple locations or viewpoints, and thus, the prerequisite of IILFM is not satisfied. 


https://user-images.githubusercontent.com/58899542/208347959-75cfc3b0-d0a8-4b5c-88ee-439d915d61bb.mp4

![fig2](https://user-images.githubusercontent.com/58899542/208348146-82e93a1e-757a-4ea0-962f-7ffd3358bc39.png)



To fill this technology gap, in this letter, we develop an enhanced IILFM for LiDARs with a steering mirror. In particular, we design a novel clustering algorithm, together with a cluster screening algorithm, to extract point clusters that could contain LiDAR fiducial markers from an unordered multi-view point cloud. Thereafter, we propose an approach to detect the fiducials from the point clusters scattered in the space. The extension from single-view point cloud to multi-view point cloud allows LiDARs with a steering mirror to be placed on a moving platform, such as a mobile robot, rather than a static tripod. Therefore, it is feasible to detect the fiducials in the 3D map built by a Simultaneous Localization and Mapping (SLAM) framework. <br>
![com](https://user-images.githubusercontent.com/58899542/208348280-e44dbb14-8fab-4982-86f7-274c55720604.png)<br>
![map](https://user-images.githubusercontent.com/58899542/208348093-d83933a1-097c-4a9b-ae4f-bea9daf40377.png)<br>
## Supplementary Instructions
The pcd file corresponding to the above video is available at [GoogleDrive](https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing). Again, please refer to [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) if you are interested in the generation of the multi-viewpoint point cloud. Our algorithm requires that the inputted point cloud is in the format of pcd. [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) is just a tool to help you obtain such a point cloud with a Livox mid-40 LiDAR rather than a necessary condition. That is, you can use other ways, for instance, [terrestrial laser scanning](https://www.youtube.com/watch?v=4-Cxoyb9N_c&t=291s), to acquire the multi-viewpoint point cloud. As long as the format is in pcd and the intensity values are available, our algorithm is applicable.

# How to use
## Requirements
* Ubuntu 20.04 <br>
Other versions of the Ubuntu system could work if the following libraries are installed correctly.<br>
* PCL <br>
``sudo apt update``<br>
``sudo apt install libpcl-dev``<br>
* OpenCV <br>
``sudo apt update``<br>
``sudo apt install libopencv-dev python3-opencv``<br>
* catkin<br>
``sudo apt update``<br>
``sudo apt install catkin``<br>
* yaml-cpp <br>
``sudo apt update``<br>
``sudo apt-get install libyaml-cpp-dev``<br>

## Commands
```git clone https://github.com/York-SDCNLab/Marker-Detection-General.git```<br>
```cd Marker-Detection-General```<br>
```cd aruco_detection```<br>
```mkdir build```<br>
```cd build```<br>
```cmake ..```<br>
Then, download [the pcd file](https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing) and put it in the ```build``` folder. <br>
Move the ```config.yaml``` file into the ```build``` folder as well. It can be found in ```aruco_detection``` folder.<br>
Run the following command in the ```build``` folder<br>
```./tag_detection```<br>
Afterewards, the visualization of the marker detection process will be shown in the 3D viewer.<br>
Moreover, the detectio result will be shwon in the terminal.<br>








