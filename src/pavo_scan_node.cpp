#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "pavo_driver.h"
#include <fstream>

#define MAX_RESPONSES (8192)
#define COMP_NODES (36000)
#define MIN_RANGE (0.15f)
#define MAX_RANGE (40.0f)

void publish_msg(ros::Publisher* pub,
		pavo_response_scan_t* nodes,
		int node_count,
		ros::Time start,
		double scan_time,
		std::string frame_id)
{
	sensor_msgs::LaserScan scanMsg;
	scanMsg.header.stamp = start;
	//scanMsg.header.frame_id = "scan_frame";
	scanMsg.header.frame_id = frame_id.c_str();

	int start_node = 0;
	while(nodes[start_node++].distance==0);
	start_node--;

	int end_node = node_count-1;
	while(nodes[end_node--].distance==0);
	end_node++;
  
	if(start_node>=end_node)
          return;  

	scanMsg.angle_min = Degree2Radians((float)nodes[start_node].angle /100.0f);
	scanMsg.angle_max = Degree2Radians((float)nodes[end_node].angle /100.0f);
	scanMsg.scan_time = scan_time;

	scanMsg.angle_increment = (scanMsg.angle_max-scanMsg.angle_min)/(end_node-start_node);
        if( scanMsg.angle_increment == 0 )
        {
          std::cout << "Zero Increment!" <<std::endl;
          return;
        }
	scanMsg.time_increment = scan_time/(end_node-start_node) ;

	scanMsg.range_min = MIN_RANGE;
	scanMsg.range_max = MAX_RANGE;

	int effective_count = end_node-start_node + 1;

	scanMsg.ranges.resize(effective_count);
	scanMsg.intensities.resize(effective_count);
	for(int i = 0; i < effective_count; ++i){
		uint16_t read_val = nodes[start_node + i].distance;
		if (read_val == 0)
			scanMsg.ranges[i] = std::numeric_limits<float>::infinity();
		else
			scanMsg.ranges[i] = read_val*0.002;
		scanMsg.intensities[i] = nodes[start_node + i].intensity;
	}

	pub->publish(scanMsg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle nh;
  std::string frame_id;
  ros::param::param<std::string>("~frame_id", frame_id, "scan_frame");

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("pavo_scan", 1000); //1000 Messages to buffer, topic: pavo_scan

  bool compensate = true;
  //bool compensate = false;
  pavo_response_scan_t* responses_ptr = new pavo_response_scan_t[MAX_RESPONSES];
  memset(responses_ptr, sizeof(pavo_response_scan_t)*MAX_RESPONSES, 0);
  int count = MAX_RESPONSES;
  int start_node = 0;
  int end_node = 0;
  
  ros::Rate rate(30); 

  pavo::pavo_driver drv;

  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;

  while(nh.ok()){
    count = MAX_RESPONSES;

    start_scan_time = ros::Time::now();
    drv.get_scanned_data(responses_ptr, count);
    //drv.get_scanned_data(responses_ptr, count, 1000);
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();
    if(compensate)
    {
      pavo_response_scan_t *comp_nodes = new pavo_response_scan_t[count];
      memset(comp_nodes, 0, sizeof(pavo_response_scan_t) * count);
      float scale = 36000.0f/count;
      for(int i =0; i<count; i++)
      {
        int angle_int = static_cast<int>(std::floor(responses_ptr[i].angle/scale));
        if(angle_int >= count)
          continue;
 
        int index = angle_int;
        comp_nodes[index].angle = static_cast<uint16_t>(std::floor(angle_int * scale));
        comp_nodes[index].distance = responses_ptr[i].distance;
        comp_nodes[index].intensity = responses_ptr[i].intensity;
      }
      publish_msg(&scan_pub, comp_nodes, count, start_scan_time, scan_duration, frame_id); //frame_id: scan_frame
      delete[] comp_nodes;
    } 
    else
    {
      publish_msg(&scan_pub, responses_ptr, count, start_scan_time, scan_duration, frame_id);
    }

    rate.sleep();
  }
}
