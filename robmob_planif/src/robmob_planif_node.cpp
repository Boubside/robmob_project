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

double resolution;
double robotRadius = 0.35;
cv::Mat map;
ros::Publisher pubPath;
nav_msgs::Path pathMsg;
bool flag = false;
int aButton = 0;
int bButton = 0;
int xButton = 0;
double xg, yg, xi, yi;
cv::Point2f robotPos;
cv::Point2f robotPosIMG;
cv::Point2f initialRobotPose;
cv::Point2f initialRobotPoseIMG;

std::vector<RRT_node> path;
cv::Point2f imageOrigin;
cv::Point2f mapOrigin;
nav_msgs::OccupancyGrid grid2;

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
  // robotPos = map2image(robot);
  return robot;

}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  aButton = joy->buttons[0];
  bButton = joy->buttons[1];
  xButton = joy->buttons[2];
}


void getmap()
{
    std::cout<<"[Planif] Map received !"<<std::endl;
    resolution = grid2.info.resolution;
    map = cv::Mat::zeros(grid2.info.height,grid2.info.width, CV_8UC1);
    std::vector<signed char> vect;
    vect = grid2.data;
    mapOrigin.x =  grid2.info.origin.position.x;
    mapOrigin.y =  grid2.info.origin.position.y;
    imageOrigin.x = -mapOrigin.x / resolution;
    imageOrigin.y = -mapOrigin.y / resolution;
    robotPos = getRobotPose();
    robotPosIMG = map2image(robotPos);
    initialRobotPoseIMG = map2image(initialRobotPose);

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
    circle(map, initialRobotPoseIMG, 5, cv::Scalar(0,0,255));
    circle(map, robotPosIMG, 5, cv::Scalar(0,255,0));

    std::cout << "Robot pose : (" << robotPos.x << ", " << robotPos.y << ")" << std::endl;
    std::cout << "Robot origin : (" << initialRobotPose.x << ", " << initialRobotPose.y << ")" << std::endl;
    std::cout << "Image origin : (" << imageOrigin.x << ", " << imageOrigin.y << ")" << std::endl;
    std::cout << "map origin : (" << mapOrigin.x << ", " << mapOrigin.y << ")" << std::endl;
    std::cout << "image size : (" << grid2.info.height << ", " << grid2.info.width << ")" << std::endl;

    std::cout<<"[Planif] Map treated !"<<std::endl;
    imshow("Map",map);
    std::cout<<"[Planif] Map displayed !"<<std::endl;
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
  ros::init(argc, argv, "robmob_planif_node");
  ros::NodeHandle _nh;
  ros::Subscriber joy_sub;
  ros::ServiceClient client;
  std::cout<<"[Planif] Planification Ready"<<std::endl;
  std::cout<<"[Planif] Press [A] to start planification" << std::endl;
  client = _nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
  joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy",10, &joyCallback);
  pubPath = _nh.advertise<nav_msgs::Path>("path", 10);

  ros::Rate rate(50);
  nav_msgs::GetMap srv;
  // if (!client.call(srv))
  // {
  //   std::cout<<"[Planif] Service call failed"<<std::endl;
  //   return 0;
  // }
  // grid2 = srv.response.map;
  initialRobotPose = getRobotPose();
  // initialRobotPose = robotPos;
  std::cout << "Robot origin : (" << initialRobotPose.x << ", " << initialRobotPose.y << ")" << std::endl;

  while(!aButton && ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"[Planif] Waiting for Map ..."<<std::endl;
  while(aButton && ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }


  if (!ros::ok()) std::cout<<"ros is not OK"<<std::endl;
  while(!aButton && ros::ok()){
    while(bButton && ros::ok()){ //release B Button
      ros::spinOnce();
      rate.sleep();
    }

    if (!client.call(srv))
    {
      std::cout<<"[Planif] Service call failed"<<std::endl;
      return 0;
    }
    grid2 = srv.response.map;
    getmap();
    RRT_tree mojo;
    std::cout << "Resolution : " << resolution << ", Robot radius : " << robotRadius / resolution << std::endl;
    path = mojo.findPath(robotPosIMG.x, robotPosIMG.y, initialRobotPoseIMG.x, initialRobotPoseIMG.y, robotRadius/resolution, map, true, 20);
    std::cout<<"[Planif] [A] Validate Path"<<std::endl;
    std::cout<<"[Planif] [B] Reboot Path"<<std::endl;
    std::cout<<"[Planif] [X] Exit Planification"<<std::endl;

    while(!bButton && ros::ok()){
      if (xButton){
        std::cout<<"[Planif] Planification Ended"<<std::endl;
        return 0;
      }
      if (aButton){
        break;
      }
      ros::spinOnce();
      rate.sleep();
    }


  }
  generatePathMessage();
  pubPath.publish(pathMsg);

  while(aButton && ros::ok()){ //release A Button
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"[Planif] Press [X] to end planification node" << std::endl;

  while(!xButton && ros::ok()){ //press X Button
    ros::spinOnce();
    rate.sleep();
  }

  std::cout<<"[Planif] Planif finished" << std::endl;
  return 0;
}
