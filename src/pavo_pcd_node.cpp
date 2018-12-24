#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pavo_driver.h"
#define MAX_RESPONSES (4096)

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  pavo_response_pcd_t* pcd_ptr = new pavo_response_pcd_t[MAX_RESPONSES];
  int num_points;

  ros::Rate rate(30.0);
  pavo::pavo_driver drv;
  unsigned int count = 0;
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    //drv.get_scanned_data(pcd_ptr, num_points);
	count = MAX_RESPONSES;
    drv.get_scanned_data(pcd_ptr, num_points, count);
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "pcd_frame";

    cloud.points.resize(num_points);

    //add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = pcd_ptr[i].x * 0.002f ;
      cloud.points[i].y = pcd_ptr[i].y * 0.002f ;
      cloud.points[i].z = pcd_ptr[i].z;
      cloud.channels[0].values[i] = pcd_ptr[i].intensity;
    }

    cloud_pub.publish(cloud);
    rate.sleep();
  }
}
