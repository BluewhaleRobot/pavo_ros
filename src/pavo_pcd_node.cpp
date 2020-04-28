#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "pavo_driver.h"
#define MAX_RESPONSES (4096)

pavo::pavo_driver *drv = NULL;
int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");
  std::string frame_id, lidar_ip, host_ip, cloud_topic;
  int lidar_port, host_port;
  int motor_speed,method;
  int merge_coef;
  double angle_min;
  double angle_max;
  bool inverted,enable_motor,switch_active_mode;
  ros::NodeHandle nh;
  
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("frame_id", frame_id, "pcd_frame");
  nh_private.param<std::string>("cloud_topic",cloud_topic,"cloud");
  nh_private.param<double>("angle_max", angle_max , 135.0);
  nh_private.param<double>("angle_min", angle_min , -135.0);
  nh_private.param<int>("motor_speed", motor_speed , 15);
  nh_private.param<int>("merge_coef", merge_coef , 2);
  nh_private.param<bool>("enable_motor", enable_motor , true);
  nh_private.param<std::string>("lidar_ip", lidar_ip , "10.10.10.101");
  nh_private.param<int>("lidar_port", lidar_port , 2368);
  nh_private.param<std::string>("host_ip", host_ip ,"10.10.10.100");
  nh_private.param<int>("host_port", host_port , 2368);
  nh_private.param<bool>("inverted", inverted , false);
  nh_private.param<int>("method", method , 0);
  nh_private.param<bool>("switch_active_mode",switch_active_mode,false);

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>(cloud_topic, 50);

  std::vector<pavo_response_pcd_t> pcd_vec;
  int num_points;

  ros::Rate rate(motor_speed);

  if(!switch_active_mode)
  {
          drv = new pavo::pavo_driver();
  }
  else
  {
          drv = new pavo::pavo_driver(host_ip,host_port);
          
  }
  drv->pavo_open(lidar_ip, lidar_port);
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;

  drv->enable_motor(enable_motor);
  if(!enable_motor)
  {
          pid_t kill_num;
          kill_num  = getppid();
          kill(kill_num,4);
          return 0;
  }

  if(!nh.getParam("/PavoPcdNode/motor_speed", motor_speed))
  {
    //set the motor speed to the defaul value 
    //when fail to get the value from the configure file
        motor_speed=15;
  }
  if(!nh.getParam("/PavoPcdNode/merge_coef", merge_coef))
  {
    //set the motor speed to the defaul value 
    //when fail to get the value from the configure file
        merge_coef=4;
  }
  ROS_INFO("motor_speed: %d",(int)motor_speed);
  ROS_INFO("merge_coef: %d",(int)merge_coef);
  
  if(method == 0 || method == 1 || method == 2 || method == 3)
  {
	drv->enable_tail_filter(method);
        if(method>0)
	ROS_INFO("success to eliminate the tail by using method: %d",(int)method);
  }
  else{
	ROS_ERROR("false to set tail filter!");
        return 0;	
  }
  
  if(drv->set_motor_speed(motor_speed)){  //设置为15Hz  
  //if(drv.set_motor_speed(15)){  //设置为15Hz  
          ROS_INFO("success to set speed!");
  }else{
          ROS_ERROR("false to set speed!");
          return -1;
  }

  if(drv->set_merge_coef(merge_coef)){  //设置1点合并,分辨率是0.12° 
  //if(drv.set_merge_coef(4)){  //设置1点合并,分辨率是0.12° 
          ROS_INFO("success to set merge!");
  }else{
          ROS_ERROR("false to set merge!");
          return -1;
  }

  unsigned int count = 0;
  while(nh.ok()){
    sensor_msgs::PointCloud cloud;
    drv->get_scanned_data(pcd_vec);
    num_points = pcd_vec.size();
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = frame_id;

    cloud.points.resize(num_points);

    //add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);
        if (!inverted)
        {
                for(unsigned int i = 0; i < num_points; i++){
                        cloud.points[i].x = -pcd_vec[i].x * 0.002f ;
                        cloud.points[i].y = -pcd_vec[i].y * 0.002f ;
                        cloud.points[i].z = pcd_vec[i].z;
                        cloud.channels[0].values[i] = pcd_vec[i].intensity;
                }       
        }
        else 
        {
                for(unsigned int i = 0; i < num_points; i++){
                        cloud.points[num_points-1-i].x = -pcd_vec[i].x * 0.002f ;
                        cloud.points[num_points-1-i].y = pcd_vec[i].y * 0.002f ;
                        cloud.points[num_points-1-i].z = pcd_vec[i].z;
                        cloud.channels[0].values[num_points-1-i] = pcd_vec[i].intensity;                 
                }
        }
        
    cloud_pub.publish(cloud);
    rate.sleep();
  }
    delete drv;
  }
