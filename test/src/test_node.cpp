#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::Subscriber map_sub;

  map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &mapCallback);

  return 0;
}
