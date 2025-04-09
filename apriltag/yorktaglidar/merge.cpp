//standard imports
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <dirent.h>

//point cloud imports
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <float.h>
#include <algorithm>
#include <iterator>
#include <vtkAutoInit.h>
#include<ctime>
#include<cstdlib>


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/range_image/impl/range_image.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/intensity_gradient.h>
#include<pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/aruco.hpp>


#include "yaml-cpp/yaml.h"
#include <Eigen/Core>


int main(int argc, char* argv[]){
	//dir vars

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile("./dense1.pcd", *cloud_1) == -1){
		PCL_ERROR("read false");

		return 0;
	}

    if (pcl::io::loadPCDFile("./dense3.pcd", *cloud_2) == -1){
		PCL_ERROR("read false");

		return 0;
	}

    Eigen::Matrix4f transform; //1st Transformation Matrix

	transform.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    Eigen::Matrix3f R_o;         								
	// R_o<<0,0,1,0,1,0,-1,0,0;
    R_o<<1,0,0,0,1,0,0,0,1;
    // R_o<<-0.6214,0.7835,0,-0.7835,-0.6214,0,0,0,1;


    Eigen::Vector3f translation(4.0, 0, 0);

    transform.block<3,3>(0,0) = R_o;
       
    transform.block<3,1>(0,3) = translation;

    pcl::transformPointCloud(*cloud_2,*cloud_2,transform);

    *cloud_3 = (*cloud_1)+(*cloud_2);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3d view"));
    
    viewer->setBackgroundColor(0.1,0.1,0.1);

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor_raw(cloud_3, "intensity");

    viewer->addPointCloud(cloud_3,fildColor_raw,"merged point cloud");
    viewer->addCoordinateSystem();

	
	pcl::io::savePCDFileASCII("./merged.pcd", *cloud_3);
	
	// while (!viewer.wasStopped())
    // {

        viewer->spin();
    // }
	return 0;
}