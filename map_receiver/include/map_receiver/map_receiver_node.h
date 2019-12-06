#ifndef MAPRECEIVER_H
#define MAPRECEIVER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

class map_receiver_node{

  private :
  ros::NodeHandle _nh;
  ros::Subscriber occupancyGrid;
  Mat map;
  int flag;

  public :
  map_receiver_node();
  ~map_receiver_node();
  void showMap();
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);

  void setFlag(int x){flag = x;}
  int getFlag(){return flag;}
};




#endif
