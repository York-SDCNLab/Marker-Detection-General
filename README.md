# Marker-Detection-General
This is the repository for our paper 'Fiducial Marker Detection in General Point Cloud with Intensity'.<br>
<br>
The existing LiDAR fiducial marker systems have apparent usage restrictions: <br>
<img width="480" height="320" src="https://user-images.githubusercontent.com/58899542/175344791-ad9a54b5-a8f1-4cd1-9a95-dfbe96c55f07.png"/> <br>
(a): The [LiDARTag](https://github.com/UMich-BipedLab/LiDARTag) system requires that there is adequate clearance around the marker's object due to its clustering method, which makes the marker an extra 3D object added to the environment. (b): The [IILFM](https://github.com/York-SDCNLab/IILFM) system adopts spherical projection to transfer the 3D point cloud to a 2D image and then carry out marker detection. As a result, in cases where the spherical projection is not applicable, the IILFM is not applicable.




With no spatial marker placement requirement, fiducial marker detection in the general point cloud remains an unsolved problem. The general point cloud here refers to, but is not limited to, the 3D map built by Simultaneous Localization and Mapping or the 3D point cloud generated from Structure from Motion. The following video shows the construction of a general point cloud from a [modified livox_mapping](https://github.com/York-SDCNLab/Modified_livox_mapping) framework.
![sucai_02_](https://user-images.githubusercontent.com/58899542/174899500-b25e7412-fe16-42eb-b0ec-b994bd12066f.gif)

In this work, we introduce a strong generic LiDAR fiducial
marker detection algorithm 




https://drive.google.com/file/d/1Ky2VkhjBpM8Guu6jKD_OapUoRiTiqcfk/view?usp=sharing
