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
int flag = 1;
int aButton;
double xg, yg, xi, yi;
cv::Point robotPos;
cv::Point center;
std::vector<RRT_node> path;

cv::Point image2map(cv::Point p);

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
  }

  // std::cout << xi << ", " << yi << std::endl;
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  aButton = joy->buttons[0];
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
  std::cout<<"mapCallback"<<std::endl;
  if (flag==0){
    std::cout<<"Map received !"<<std::endl;
    resolution = grid->info.resolution;
    map = cv::Mat::zeros(grid->info.height,grid->info.width, CV_8UC1);
    std::vector<signed char> vect;
    vect = grid->data;
    center.x = - grid->info.origin.position.x / resolution;
    center.y = - grid->info.origin.position.y/ resolution;
    getRobotPose();
    robotPos.x = robotPos.x/resolution + center.x;
    robotPos.y = robotPos.y/resolution + center.y;
    std::cout<<"Résolution : "<<resolution<<std::endl;
    std::cout<<"Origin :"<<std::endl<<"x : "<<center.x<<std::endl<<"y : "<<center.y<<std::endl;

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
    circle(map, center, 2, cv::Scalar(0,0,255));
    circle(map, robotPos, 2, cv::Scalar(0,255,0));


    // std::cout << "x = " << xi << ", y = " << yi << std::endl;

    std::cout<<"Map treated !"<<std::endl;
    imshow("Map",map);

    RRT_tree mojo;
    path = mojo.findPath(robotPos.x, robotPos.y, center.x, center.y, robotRadius/resolution, map, true);
    flag = 1;
  }
}

void generatePathMessage()
{
  geometry_msgs::PoseStamped pose;
  for(size_t i = 0; i < path.size(); i++)
  {
    cv::Point inImagePoint(path[i].getX(),path[i].getY());
    cv::Point inMapPoint = image2map(inImagePoint);
    pose.pose.position.x = inMapPoint.x;
    pose.pose.position.x = inMapPoint.y;
    pathMsg.poses.push_back(pose);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robmob_planif_node");
  ros::NodeHandle _nh;
  ros::Subscriber map_sub;
  ros::Subscriber joy_sub;
  aButton = 0;
  std::cout<<"Hello MineTurtle"<<std::endl;

  map_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &mapCallback);
  joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy",10, &joyCallback);
  pubPath = _nh.advertise<nav_msgs::Path>("path", 10);

  while(!aButton && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }
  flag = 0;
  std::cout<<"Middle MineTurtle !"<<std::endl;

  while(aButton && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }


  while(!aButton && ros::ok()){
    ros::spinOnce();
    cv::waitKey(100);
  }

  pubPath.publish(pathMsg);

  std::cout<<"Goodbye MineTurtle"<<std::endl;
  return 0;
}
