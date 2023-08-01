# Motivations

The LiDAR fiducial marker, akin to the well-known AprilTag used in camera applications, serves as a convenient resource to impart artificial features to the LiDAR sensor, facilitating robotics applications. Unfortunately, current LiDAR fiducial marker detection methods are limited to occlusion-free point clouds. In this work, we present a novel approach for occlusion-resistant LiDAR fiducial marker detection. The following video shows a point cloud with occlusion. It is a 3D map built from the [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping).

https://user-images.githubusercontent.com/58899542/208347959-75cfc3b0-d0a8-4b5c-88ee-439d915d61bb.mp4

![fig2](https://user-images.githubusercontent.com/58899542/208348146-82e93a1e-757a-4ea0-962f-7ffd3358bc39.png)

![map](https://user-images.githubusercontent.com/58899542/208348093-d83933a1-097c-4a9b-ae4f-bea9daf40377.png)<br>
![image](https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/b319bf13-b622-4044-adf9-1fddca7d6c25)<br>

![image](https://github.com/York-SDCNLab/Marker-Detection-General/assets/58899542/6a8cd9db-1264-4df1-bfe4-e981d8991176)<br>
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








