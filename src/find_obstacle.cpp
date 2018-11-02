#include "find_obstacles.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_obstacle");
  ObstacleFinder of;
  ros::spin();
  return 0;
}
