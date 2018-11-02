#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "move_car.h"

/**
   Class that subscribes to the topic giving the position of the closest obstacle
   in the body fixed coordinate system with x pointing forward and y pointing left.
 */
class ObstacleSeeker
{
  ros::NodeHandle nh_;
  ros::Subscriber obstacle_sub_;

  AckermannMover am;

  float Kd_;
  float Ka_;

 public:
  ObstacleSeeker(float Kd, float Ka): Kd_(Kd), Ka_(Ka) {
    obstacle_sub_ = nh_.subscribe("/obstacle", 1,
			      &ObstacleSeeker::pointCallback, this);
  }
  ~ObstacleSeeker(){}

  /**
     Callback function that moves the car towards the obstacle
  */
  void pointCallback(const geometry_msgs::Point::ConstPtr& point){
    // If distance to the obstacle is less than a meter, stop
    float dist = point->z;
    float speed = 0;
    float theta = 0;
    
    if (dist > 1) {
      // Steer towards the obstacle
      theta = std::min(0.4, std::max(-0.4, Ka_*atan2(point->y, point->x))); // Clamped to +/- 27deg
      speed = (dist-1)*Kd_;
    }

    ROS_INFO("Sending command speed=%f, steering_angle=%f", speed, theta);
    am.go(speed, theta);
  }
};


int main(int argc, char** argv)
{

  float Kd = 0.1;
  float Ka = 2;
  
  if (argc > 1) { Kd = atof(argv[1]); }
  if (argc > 2) { Ka = atof(argv[2]); }

  ros::init(argc, argv, "goto_obstacle");
  ObstacleSeeker os(Kd, Ka);
  ros::spin();
  return 0;
}


