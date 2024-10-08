# Motivations

The paper is published in [IEEE ACCESS](https://ieeexplore.ieee.org/abstract/document/10654791).

The LiDAR fiducial tag, akin to the well-known AprilTag used in camera applications, serves as a convenient resource to impart artificial features to the LiDAR sensor, facilitating robotics applications. Unfortunately, the existing LiDAR fiducial tag localization methods do not apply to 3D LiDAR maps while resolving this problem is beneficial to LiDAR-based relocalization and navigation. In this paper, we develop a novel approach to directly localize fiducial tags on a 3D LiDAR prior map, returning the tag poses (labeled by ID number) and vertex locations (labeled by index) w.r.t. the global coordinate system of the map. The following shows a 3D map built from the [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping).



![fig2](https://user-images.githubusercontent.com/58899542/208348146-82e93a1e-757a-4ea0-962f-7ffd3358bc39.png) <br>

![map](https://user-images.githubusercontent.com/58899542/208348093-d83933a1-097c-4a9b-ae4f-bea9daf40377.png)<br>
Our method can detect the LiDAR fiducial markers in a point cloud with occlusion while computing their poses relative to the global frame. <br>
<img width="600" height="150" src="https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/b319bf13-b622-4044-adf9-1fddca7d6c25"/> <br>

The following is a synthesis point cloud with occlusion. The two presenters are holding two different AprilTag markers. Observing along the X-axis of the global coordinate system, the back subpoint cloud is totally blocked by the front one. Thus, it is infeasible to project the 3D point cloud to a 2D image plane due to the occlusion.


https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/fd501204-be6a-4c09-a6cb-46e6f2b03f1a



![process](https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/da0baa0f-f278-4296-a7e2-6d8804038a32)



https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/c0e42375-84de-405a-adf8-dceab31a88e1


<img width="600" height="200" src="https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/6a8cd9db-1264-4df1-bfe4-e981d8991176"/> <br>

Besides the capability of handling point clouds with occlusion, our method shows superiority in terms of pose estimation accuracy compared to [IILFM](https://github.com/York-SDCNLab/IILFM).
![image](https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/5fa06a1b-a7af-40f2-9478-b67543dfbe02)

## Supplementary Instructions
The pcd file corresponding to the above video is available at [GoogleDrive](https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing). Again, please refer to [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) if you are interested in the generation of the point cloud. Our algorithm requires that the inputted point cloud is in the format of pcd. [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) is just a tool to help you obtain such a point cloud with a Livox mid-40 LiDAR rather than a necessary condition. That is, you can use other ways, for instance, [terrestrial laser scanning](https://www.youtube.com/watch?v=4-Cxoyb9N_c&t=291s), to acquire a point cloud. As long as the format is in pcd and the intensity values are available, our algorithm is applicable.

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
Afterward, the visualization of the marker detection process will be shown in the 3D viewer.<br>
Moreover, the detection result will be shown in the terminal.<br>

# Citation
If you find this work helpful for your research, please cite our paper:
```
@ARTICLE{10654791,
  author={Liu, Yibo and Shan, Jinjun and Schofield, Hunter},
  journal={IEEE Access}, 
  title={Improvements to Thin-Sheet 3D LiDAR Fiducial Tag Localization}, 
  year={2024},
  volume={12},
  number={},
  pages={124907-124914},
  keywords={Three-dimensional displays;Laser radar;Point cloud compression;Simultaneous localization and mapping;Visualization;Algorithm design and theory;Fiducial tag;LiDAR;Localization},
  doi={10.1109/ACCESS.2024.3451404}}
```








