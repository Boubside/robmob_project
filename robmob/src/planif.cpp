#include "robmob_planif/RRT_tree.hpp"
#include "ros/console.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>


bool flag = false;
double resolution;

cv::Mat map;
cv::Point2f robotPos;
cv::Point2f robotPosIMG;
cv::Point2f initialRobotPose;
cv::Point2f initialRobotPoseIMG;

cv::Point2f imageOrigin;
cv::Point2f mapOrigin;

std::vector<RRT_node> path;
ros::Publisher pubPath;
nav_msgs::Path pathMsg;

cv::Point2f map2image(cv::Point2f mapPoint){
  cv::Point2f imgPoint;
  imgPoint.x = mapPoint.x/resolution + imageOrigin.x;
  imgPoint.y = mapPoint.y/resolution + imageOrigin.y;
  return imgPoint;
}

cv::Point2f image2map(cv::Point2f imgPoint){
  cv::Point2f mapPoint;
  mapPoint.x = imgPoint.x * resolution + mapOrigin.x;
  mapPoint.y = imgPoint.y * resolution + mapOrigin.y;
  return mapPoint;
}

void keyboardCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if(msg->linear.y < 0 && !flag) flag = true;
  else if(msg->linear.y < 0 && flag) flag = false;
}

cv::Point2f getRobotPose(){
  tf::TransformListener listener;
  bool found = false;
  ros::Time now = ros::Time(0);
  tf::StampedTransform transform;
  while(ros::ok() && !found){
    try{
      listener.waitForTransform("/map","/base_link",now,ros::Duration(2.0));
      listener.lookupTransform("/map","/base_link",now,transform);
      found = true;
    }
    catch(tf::ExtrapolationException e){
      ROS_INFO("%s", e.what());
      ros::Duration(1.0).sleep();
    }
    catch(tf::TransformException e){
      ROS_INFO("%s", e.what());
      ros::Duration(1.0).sleep();
    }
  }
  cv::Point2f robot;
  robot.x = transform.getOrigin().x();
  robot.y = transform.getOrigin().y();
  return robot;
}

void getmap(nav_msgs::OccupancyGrid grid)
{
  map = cv::Mat::zeros(grid.info.height,grid.info.width, CV_8UC1);

  mapOrigin.x =  grid.info.origin.position.x;
  mapOrigin.y =  grid.info.origin.position.y;
  imageOrigin.x = -mapOrigin.x / resolution;
  imageOrigin.y = -mapOrigin.y / resolution;

  robotPos = getRobotPose();
  robotPosIMG = map2image(robotPos);
  initialRobotPoseIMG = map2image(initialRobotPose);
  std::vector<signed char> vect = grid.data;

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
  cv::Mat temp = map.clone();
  circle(temp, initialRobotPoseIMG, 3, cv::Scalar(0,0,255));
  circle(temp, robotPosIMG, 3, cv::Scalar(0,255,0));

  imshow("Map", temp);
  cv::waitKey(0);

  std::cout << "Resolution : " << grid.info.resolution << std::endl;
  std::cout << "size : " << grid.info.height << ", " << grid.info.width << std::endl;
  std::cout << "origin : " << grid.info.origin.position.x << ", " << grid.info.origin.position.y << std::endl;
}

void generatePathMessage()
{
  if(path.empty()) return;
  std::cout << "Path size is " << path.size() << std::endl;
  geometry_msgs::PoseStamped pose;
  for(size_t i = 1; i < path.size(); i++)
  {
    cv::Point2f inImagePoint(path[i].getX(),path[i].getY());
    cv::Point2f inMapPoint = image2map(inImagePoint);
    pose.pose.position.x = inMapPoint.x;
    pose.pose.position.y = inMapPoint.y;
    pathMsg.poses.push_back(pose);
  }
}

int main(int argc, char **argv)
{
  double robotRadius = 0.35;

  ros::init(argc, argv, "planif_node");
  ros::NodeHandle nh;
  ros::Rate rate(100);
  nav_msgs::OccupancyGrid grid;

  ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
  nav_msgs::GetMap srv;
  ros::Subscriber teleop = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10, &keyboardCallback);
  pubPath = nh.advertise<nav_msgs::Path>("path", 10);

  initialRobotPose = getRobotPose();
  std::cout << "Initial pose : " << initialRobotPose.x << "," << initialRobotPose.y << std::endl;

  while(!flag && ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  if (client.call(srv))
  {
    std::cout<<"service call worked"<<std::endl;
  }
  else
  {
    std::cout << "service call failed" << std::endl;
    return 0;
  }

  grid = srv.response.map;
  resolution = grid.info.resolution;
  getmap(grid);
  RRT_tree mojo;
  std::cout << "Resolution : " << resolution << ", Robot radius : " << robotRadius / resolution << std::endl;
  path = mojo.findPath(robotPosIMG.x, robotPosIMG.y, initialRobotPoseIMG.x, initialRobotPoseIMG.y, robotRadius/resolution, map, true, 20);
  cv::waitKey(0);

  generatePathMessage();
  pubPath.publish(pathMsg);

  while(flag && ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
