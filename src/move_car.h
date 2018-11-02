#ifndef MOVE_CAR_H
#define MOVE_CAR_H
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Header.h>


class AckermannMover
{
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;  
public:
  AckermannMover() {
    cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 10);
  }

  ~AckermannMover()  { }

  void go(float speed, float steering_angle) {
    //std_msgs::Header header;
    //ackermann_msgs::AckermannDrive drive;
    ackermann_msgs::AckermannDriveStamped driveStamped;
    driveStamped.drive.steering_angle = steering_angle;
    driveStamped.drive.speed = speed;

    cmd_pub_.publish(driveStamped);
  }
};

#endif
