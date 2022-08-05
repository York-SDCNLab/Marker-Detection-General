# Background
This is the repository for our paper 'Fiducial Marker Detection in Multi-Viewpoint Point Cloud'.<br>
<br>
The existing LiDAR fiducial marker systems have usage restrictions: <br>
<img width="480" height="320" src="https://user-images.githubusercontent.com/58899542/175344791-ad9a54b5-a8f1-4cd1-9a95-dfbe96c55f07.png"/> <br>
(a): The [LiDARTag](https://github.com/UMich-BipedLab/LiDARTag) system requires that there is adequate clearance around the marker's object due to its clustering method, which makes the marker an extra 3D object added to the environment. (b): The [IILFM](https://github.com/York-SDCNLab/IILFM) system adopts spherical projection (it requires that the point cloud is a one-viewpoint point cloud) to transfer the 3D point cloud to a 2D image and then carry out marker detection. As a result, in cases where the spherical projection is not applicable, the IILFM is not applicable.




With no spatial marker placement requirement, fiducial marker detection in the multi-viewpoint point cloud remains an unsolved problem. The multi-viewpoint point cloud 
indicates that the points of the point cloud are sampled from multiple viewpoints. In real-world applications, the multi-viewpoint point cloud can be a 3D point cloud map built by the SLAM framework, a 3D point cloud model reconstructed by the SFM framework or just a stitch of multiple one-viewpoint point clouds. The following video shows the construction of a multi-viewpoint point cloud using the [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) framework.
![sucai_02_](https://user-images.githubusercontent.com/58899542/174899500-b25e7412-fe16-42eb-b0ec-b994bd12066f.gif)
## Supplementary Instructions
The pcd file corresponding to the above video is available [here](https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing). Again, please refer to [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) if you are interested in the generation of the multi-viewpoint point cloud. Our algorithm requires that the inputted point cloud is in the format of pcd. [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) is just a tool to help you obtain such a point cloud with a Livox mid-40 LiDAR rather than a necessary condition. That is, you can use other ways to acquire the multi-viewpoint point cloud. As long as the format is in pcd, our algorithm is applicable.
# Abstract
In this work,  we develop a novel algorithm to detect the fiducial markers in the multi-viewpoint point cloud. The proposed algorithm includes two stages. First, Regions of Interest (ROIs) detection finds point clusters that could contain fiducial markers. Specifically, a method extracting the ROIs from the intensity perspective is introduced on account of the fact that from the spatial perspective, the markers, which are sheets of paper or thin boards, are non-distinguishable from the planes to which they are attached. Second, marker detection verifies if the candidate ROIs contain fiducial markers and outputs the ID numbers and vertices locations of the markers in the valid ROIs. In particular, the ROIs are transmitted to a predefined intermediate plane for the purpose of adopting a spherical projection to generate the intensity image, and then, marker detection is completed through the intensity image.
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
```git clone https://github.com/York-SDCNLab/IILFM.git```<br>
```cd IILFM```<br>
```catkin build```<br>



