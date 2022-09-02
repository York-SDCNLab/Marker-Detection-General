This is the repository for our paper 'Fiducial Marker Detection in Multi-Viewpoint Point Cloud'. This work has been submitted to IEEE RA-L. <br>
Also, this is a follow-up work of our previously proposed [intensity image-based LiDAR fiducial marker system](https://github.com/York-SDCNLab/IILFM). <br>
The introduction video is available at [YouTube](https://www.youtube.com/watch?v=K3Mgo319mas) and [Bilibili](https://www.bilibili.com/video/BV1TN4y157JG?spm_id_from=333.999.0.0&vd_source=6ecb163024bda9a121cdd47cd37f162b).
# Background

The existing LiDAR fiducial marker systems have usage restrictions: <br>
<img width="480" height="320" src="https://user-images.githubusercontent.com/58899542/175344791-ad9a54b5-a8f1-4cd1-9a95-dfbe96c55f07.png"/> <br>
(a): The [LiDARTag](https://github.com/UMich-BipedLab/LiDARTag) system requires that there is adequate clearance around the marker's object due to its clustering method, which makes the marker an extra 3D object added to the environment. (b): The [IILFM](https://github.com/York-SDCNLab/IILFM) system adopts spherical projection (it requires that the point cloud is a one-viewpoint point cloud) to transfer the 3D point cloud to a 2D image and then carry out marker detection. As a result, in cases where the spherical projection is not applicable, the IILFM is not applicable.




With no spatial marker placement requirement, fiducial marker detection in the multi-viewpoint point cloud remains an unsolved problem. The multi-viewpoint point cloud 
indicates that the points of the point cloud are sampled from multiple viewpoints. In real-world applications, the multi-viewpoint point cloud can be a 3D point cloud map built by the SLAM framework, a 3D point cloud model reconstructed by the SFM framework or just a stitch of multiple one-viewpoint point clouds. The following video shows the construction of a multi-viewpoint point cloud using the [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) framework.
![sucai_02_](https://user-images.githubusercontent.com/58899542/174899500-b25e7412-fe16-42eb-b0ec-b994bd12066f.gif)
## Supplementary Instructions
The pcd file corresponding to the above video is available at [GoogleDrive](https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing). Again, please refer to [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) if you are interested in the generation of the multi-viewpoint point cloud. Our algorithm requires that the inputted point cloud is in the format of pcd. [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) is just a tool to help you obtain such a point cloud with a Livox mid-40 LiDAR rather than a necessary condition. That is, you can use other ways, for instance, [terrestrial laser scanning](https://www.youtube.com/watch?v=4-Cxoyb9N_c&t=291s), to acquire the multi-viewpoint point cloud. As long as the format is in pcd and the intensity values are available, our algorithm is applicable.
# Abstract
In this work,  we develop a novel algorithm to detect the fiducial markers in the multi-viewpoint point cloud. The proposed algorithm includes two stages. First, Regions of Interest (ROIs) detection finds point clusters that could contain fiducial markers. Specifically, a method extracting the ROIs from the intensity perspective is introduced on account of the fact that from the spatial perspective, the markers, which are sheets of paper or thin boards, are non-distinguishable from the planes to which they are attached. Second, marker detection verifies if the candidate ROIs contain fiducial markers and outputs the ID numbers and vertices locations of the markers in the valid ROIs. In particular, the ROIs are transmitted to a predefined intermediate plane for the purpose of adopting a spherical projection to generate the intensity image, and then, marker detection is completed through the intensity image.<br>
The following image shows the overall pipeline of the proposed algorithm.
![pipeline](https://user-images.githubusercontent.com/58899542/183443196-473c77fe-8800-4b97-902b-f6f97c85fc07.png)


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


https://user-images.githubusercontent.com/58899542/183441545-0bfe7d7e-33a5-4524-80dd-fc7e4c525bcf.mp4


Moreover, the detectio result will be shwon in the terminal.<br>
<img width="480" height="170" src="https://user-images.githubusercontent.com/58899542/183432557-e8b02010-3de9-4779-ab85-ebc908c7f388.png"/> <br>
# Comparison with the IILFM system
Compared with our previously proposed intensity image-based LiDAR fiducial marker system ([GitHub](https://github.com/York-SDCNLab/IILFM), [Paper](https://ieeexplore.ieee.org/document/9774900)), the approach introduced in this work aims at solving fiducial marker detection in the multi-viewpoint point cloud (the spherical projection is not directly applicable in this kind of point cloud). The following table shows the comparison of the two systems in terms of the accuracy.  IILFM and the proposed algorithm output similar vertices estimation results and the accuracy is similar. <br>
![image](https://user-images.githubusercontent.com/58899542/183443852-db987b38-0a52-4842-a975-327ada1180d3.png) <br>
The experimental setup is shown in the following figure.<br>
<img width="480" height="200" src="https://user-images.githubusercontent.com/58899542/183447235-86c7a9e5-916e-483a-a81f-ddf44b4070e4.png"/> <br>
Please refer to our paper for the details of the setup and the analysis of the comparison.






