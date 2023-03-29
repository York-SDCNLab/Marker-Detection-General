# Motivations

LiDAR fiducials localization on a 3D prior map is favorable to downstream tasks, including relocalization and navigation. The existing methods cannot be directly applied to this problem as they require an occlusion-free  single-view point cloud as the input. In this paper, we develop a novel method to localize fiducials, which refer to planar markers (e.g. AprilTag and ArUco), on a 3D prior map, built from a LiDAR-based Simultaneous Localization and Mapping framework, with multiple viewpoints and occlusion. In particular, we adopt the [Intensity Image-based LiDAR Fiducial Marker (IILFM)](https://github.com/York-SDCNLab/IILFM) as the baseline and develop a new fiducials localization pipeline which is composed of three steps: 1) intensity feature extraction; 2) feature clustering and cluster screening; 3) marker detection via an intermediate plane. In addition, we develop a new algorithm to boost the pose estimation accuracy over IILFM by improving the imaging quality of the intensity image. Both qualitative and quantitative experiments are conducted to demonstrate the superiority of the proposed method over the previous approaches. 

https://user-images.githubusercontent.com/58899542/208347959-75cfc3b0-d0a8-4b5c-88ee-439d915d61bb.mp4

![fig2](https://user-images.githubusercontent.com/58899542/208348146-82e93a1e-757a-4ea0-962f-7ffd3358bc39.png)

![map](https://user-images.githubusercontent.com/58899542/208348093-d83933a1-097c-4a9b-ae4f-bea9daf40377.png)<br>
![image](https://user-images.githubusercontent.com/58899542/225072073-30f40010-2c8e-4f1d-afb3-9ea603541bc7.png)
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








