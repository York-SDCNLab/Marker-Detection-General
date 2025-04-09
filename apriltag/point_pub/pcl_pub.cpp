#include <iostream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "color");  //初始化了一个节点，名字为color
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/color_cloud", 1000);  //建立了一个发布器，主题是color_cloud，方便之后发布点云

  ros::Rate rate(2);  //点云更新频率2Hz
  unsigned int num_points = 100;  //点云大小为100
  pcl::PointCloud<pcl::PointXYZRGB> cloud;  //建立了一个pcl的点云（不能直接发布）

  cloud.points.resize(num_points); //点云初始化
  
  sensor_msgs::PointCloud2 output_msg;  //建立一个可以直接发布的点云

  while (ros::ok()) {
    output_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < num_points; i++) {
      //点云中每个点位于一个10*10*10的方块内随机分布，颜色也随机
      cloud.points[i].x = 10 * rand () / (RAND_MAX + 1.0f); // rand () / (RAND_MAX + 1.0f)为[0.1)   rand () / (RAND_MAX )为[0.1]              
      cloud.points[i].y = 10 * rand () / (RAND_MAX + 1.0f);
      cloud.points[i].z = 10 * rand () / (RAND_MAX + 1.0f);
      cloud.points[i].r =  (rand() % (255+1)); //(rand() % (b-a+1))+ a
      cloud.points[i].g =  (rand() % (255+1));
      cloud.points[i].b =  (rand() % (255+1));
    }
    pcl::toROSMsg(cloud, output_msg);  //将点云转化为消息才能发布
    output_msg.header.frame_id = "map";  //frame_id为map，rviz中就不用改了
    pub.publish(output_msg); //发布出去
    rate.sleep();
  }
  ros::spin();

  return 0;
}

