#include "robmob_planif/RRT_tree.hpp"
#include "ros/console.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>

double resolution;
double robotRadius = 0.250;
cv::Mat map;
ros::Publisher pubPath;
nav_msgs::Path pathMsg;
bool flag = false;
int aButton;
double xg, yg, xi, yi;
cv::Point robotPos;
cv::Point mapOrigin;

cv::Point map2image(cv::Point mapPoint){
  cv::Point imgPoint;
  imgPoint.x = mapPoint.x/resolution + mapOrigin.x;
  imgPoint.y = mapPoint.y/resolution + mapOrigin.y;
  return imgPoint;
}

cv::Point image2map(cv::Point imgPoint){
  cv::Point mapPoint;
  mapPoint.x = (imgPoint.x + mapOrigin.x) * resolution;
  mapPoint.y = (imgPoint.y + mapOrigin.y) * resolution;
  return mapPoint;
}

void getRobotPose(){
  tf::TransformListener listener;
  bool found = false;
  while(ros::ok() && !found){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/map","/base_footprint",ros::Time(0),ros::Duration(2.0));
      listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
      found = true;
    }
    catch(tf::TransformException e){
      ROS_INFO("%s", e.what());
      ros::Duration(1.0).sleep();
    }
    robotPos.x = transform.getOrigin().x();
    robotPos.y = transform.getOrigin().y();
    robotPos = map2image(robotPos);
  }

}
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  aButton = joy->buttons[0];
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
  // std::cout<<"mapCallback"<<std::endl;
  if (flag){
    std::cout<<"Map received !"<<std::endl;
    resolution = grid->info.resolution;
    map = cv::Mat::zeros(grid->info.height,grid->info.width, CV_8UC1);
    std::vector<signed char> vect;
    vect = grid->data;
    mapOrigin.x = - grid->info.origin.position.x / resolution;
    mapOrigin.y = - grid->info.origin.position.y/ resolution;
    getRobotPose();

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
    cv::cvtColor(map,map,CV_GRAY2BGR);
    circle(map, mapOrigin, 2, cv::Scalar(0,0,255));
    circle(map, robotPos, 2, cv::Scalar(0,255,0));

    std::cout<<"Map treated !"<<std::endl;
    imshow("Map",map);


    flag = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robmob_planif_node");
  ros::NodeHandle _nh;
  ros::Subscriber map_sub;
  ros::Subscriber joy_sub;
  aButton = 0;
  std::cout<<"Planification Ready"<<std::endl;

  map_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &mapCallback);
  joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy",10, &joyCallback);
  pubPath = _nh.advertise<nav_msgs::Path>("path", 10);

  while(!aButton && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }
  flag = true;
  std::cout<<"Waiting for Map ..."<<std::endl;

  while(flag && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }

  RRT_tree mojo;
  std::vector<RRT_node> path = mojo.findPath(robotPos.x, robotPos.y, mapOrigin.x, mapOrigin.y, robotRadius/resolution, map, true);


  while(!aButton && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }

  std::cout<<"Planification ended"<<std::endl;
  return 0;
}
