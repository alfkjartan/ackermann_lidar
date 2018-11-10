#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

ros::Publisher LEFT_PUB;
ros::Publisher RIGHT_PUB;

void readAngle(const sensor_msgs::JointState::ConstPtr& joint_state) {
    float left_wheel_angle = joint_state->position[std::find (joint_state->name.begin(),joint_state->name.end(), "left_rear_axle") - joint_state->name.begin()];
    float right_wheel_angle = joint_state->position[std::find (joint_state->name.begin(),joint_state->name.end(), "right_rear_axle") - joint_state->name.begin()];

    std_msgs::Float32 left_msg;
    left_msg.data = left_wheel_angle;
    std_msgs::Float32 right_msg;
    right_msg.data = right_wheel_angle;

    LEFT_PUB.publish(left_msg);
    RIGHT_PUB.publish(right_msg);
    
    //ROS_INFO("Left wheel angle: %f", left_wheel_angle);
    //ROS_INFO("Right wheel angle: %f", right_wheel_angle);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_wheel_angles");
  ros::NodeHandle nh_;
  LEFT_PUB = nh_.advertise<std_msgs::Float32>("/left_wheel_angle", 10);
  RIGHT_PUB = nh_.advertise<std_msgs::Float32>("/right_wheel_angle", 10);
    
  ros::Subscriber sub_ = nh_.subscribe("/ackermann_vehicle/joint_states", 10, readAngle);

  ros::spin();
  return 0;
}
