#include "robmob_planif/RRT_tree.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

double resolution;
double robotRadius = 0.250;
cv::Mat map;
ros::Subscriber occupancyGrid;
int flag;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid){
  // if (flag==0){
  //   cout<<"Map received !"<<endl;
  //   resolution = grid->info.resolution;
  //   flag = 1;
  // }

  if (flag==0){
    std::cout<<"Map received !"<<std::endl;
    resolution = grid->info.resolution;
    map = cv::Mat::zeros(grid->info.height,grid->info.width, CV_8UC1);
    std::vector<signed char> vect;
    vect = grid->data;

    for (int i = 0; i< map.rows; i++){
      for (int j = 0; j< map.cols; j++){
        if (vect[i*map.cols + j] == -1)
          map.at<uchar>(i,j) = 150;
        if (vect[i*map.cols + j] == 0)
          map.at<uchar>(i,j) = 255;
        if (vect[i*map.cols + j] == 100)
          map.at<uchar>(i,j) = 0;
      }
    }
    std::cout<<"Map treated !"<<std::endl;
    imshow("Map",map);
    flag = 1;
  }
}


int main(int argc, char **argv){
  flag = 0;

  ros::init(argc, argv, "robmob_planif_node");
  ros::NodeHandle _nh;
  occupancyGrid = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &mapCallback);
  char key = 'a';

  while(flag != 1 && key != 'q'){
    ros::spinOnce();
    std::cout << "flag : " << flag << std::endl;
  }

  int radius = ceil(robotRadius / resolution);
  std::cout << "Radius : " << radius << std::endl;
  RRT_tree t;
  // cv::Mat map = cv::imread("/home/sj/map.jpg");
  // if(! map.data )                              // Check for invalid input
  // {
  //   std::cout <<  "Could not open or find the image" << std::endl ;
  //   return -1;
  // }
  cv::cvtColor(map,map,CV_GRAY2BGR);

  std::cout << "Starting rrt" << std::endl;
  std::vector<RRT_node> path = t.findPath(50, 50, 550, 550, radius, map, true, 20);
  std::cout << "rrt finished" << std::endl;

  cv::waitKey(0);

  return 0;
}
