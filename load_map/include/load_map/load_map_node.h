#ifndef LOADMAP_H
#define LOADMAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/package.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>

using namespace cv;
using namespace std;

class load_map_node{

public :
    load_map_node();
    ~load_map_node();
    int getMap();
    void processMap();
    void sendMap();

private :
    ros::NodeHandle _nh;
    std::string path;
    Mat map;
    Mat procMap;
    vector<signed char> data;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid grid;


};

#endif
