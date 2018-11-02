#ifndef FIND_OBSTACLES_H
#define FIND_OBSTACLES_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

/**
   Class that subscribes to the laser scan data from the ackermann vehicle,
   and implements some basic algorithms to find targets/obstacles
   The class is header-file only. You only need to include the header file in your code.
 */
class ObstacleFinder
{
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher obstacle_pub_;
  
 public:
  ObstacleFinder(){
    scan_sub_ = nh_.subscribe("/ackermann_vehicle/laser/scan", 1,
			      &ObstacleFinder::laserCallback, this);
    obstacle_pub_ = nh_.advertise<geometry_msgs::Point>("/obstacle", 10);
  }
  ~ObstacleFinder(){}

  /**
     Callback function analyzing the incoming laser scans
  */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {

    
    
    float near_lim = 0.4;
    float far_lim = scan->range_max;
    
    // Find the minimim range reading that is > near_lim
    int range_index;
    float range = far_lim;
    int k = 0;
    for (std::vector<float>::const_iterator it = scan->ranges.begin();
	 it != scan->ranges.end(); ++it) {
      if ( (*it > near_lim) and (*it < far_lim) ) {
	if (*it < range) {
	  range = *it;
	  range_index = k;
	  //ROS_INFO("Found range %f, at index %d", *it, k);
	}
      }
      k++;
    }
    
    // The angle corresponding to the closest 
    float angle = range_index * scan->angle_increment;
    // This angle is CLOCKWISE from the direction of the first sample which is
    // FORWARD. This corresponds to the default of the rplidar,
    // which is supposed to be mounted with  cable towards the back.

    // Publish the position of the closest obstacle as a point in local tf with x pointing forward
    // and y to the left.
    geometry_msgs::Point obstacle_msg;
    obstacle_msg.x = range*cos(angle);
    obstacle_msg.y = -range*sin(angle);
    obstacle_msg.z = range;
    obstacle_pub_.publish(obstacle_msg);
    
  }

  };

#endif 
