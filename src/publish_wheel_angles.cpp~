#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


void readAngle(const sensor_msgs::JointState::ConstPtr& joint_state) {
    float left_wheel_angle = joint_state->position[std::find (joint_state->name.begin(),joint_state->name.end(), "left_rear_axle") - joint_state->name.begin()];
    float right_wheel_angle = joint_state->position[std::find (joint_state->name.begin(),joint_state->name.end(), "right_rear_axle") - joint_state->name.begin()];

    ROS_INFO("Left wheel angle: %f", left_wheel_angle);
    ROS_INFO("Right wheel angle: %f", right_wheel_angle);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_whee_angles");
  ros::NodeHandle nh_;
  ros::Subscriber sub_ = nh_.subscribe("/ackermann_vehicle/joint_states", 10, readAngle);

  ros::spin();
  return 0;
}
