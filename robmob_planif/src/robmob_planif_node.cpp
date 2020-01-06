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

double resolution;
double robotRadius = 0.250;
cv::Mat map;
ros::Subscriber occupancyGrid;
ros::Publisher pubPath;
nav_msgs::Path pathMsg;
int flag;
double xg, yg, xi, yi;

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
    xi = transform.getOrigin().x();
    yi = transform.getOrigin().y();
  }

  std::cout << xi << ", " << yi << std::endl;
}

// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid){
//   if (flag==0){
//     std::cout<<"Map received !"<<std::endl;
//     resolution = grid->info.resolution;
//     map = cv::Mat::zeros(grid->info.height,grid->info.width, CV_8UC1);
//     std::vector<signed char> vect;
//     vect = grid->data;
//
//     for (int i = 0; i< map.rows; i++){
//       for (int j = 0; j< map.cols; j++){
//         if (vect[i*map.cols + j] == -1)
//           map.at<uchar>(i,j) = 150;
//         if (vect[i*map.cols + j] == 0)
//           map.at<uchar>(i,j) = 255;
//         if (vect[i*map.cols + j] == 100)
//           map.at<uchar>(i,j) = 0;
//       }
//     }
//
//     getRobotPose();
//
//     std::cout << "x = " << xi << ", y = " << yi << std::endl;
//
//     std::cout<<"Map treated !"<<std::endl;
//     imshow("Map",map);
//     flag = 1;
//   }
// }
//
//
//
// void createPathMsg(std::vector<RRT_node>& path){
//   geometry_msgs::PoseStamped p;
//   for(int i = 0; i < path.size(); i++){
//     p.pose.position.x = path[i].getX();
//     p.pose.position.y = path[i].getY();
//     pathMsg.poses.push_back(p);
//   }
// }


void getMap(){
  
}

int main(int argc, char **argv){

  ros::init(argc, argv, "robmob_planif_node");





  getRobotPose();

  ros::spin();

  return 0;

  // flag = 0;
  //
  // ros::init(argc, argv, "robmob_planif_node");
  // ros::NodeHandle _nh;
  // getRobotPose();
  // occupancyGrid = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 10, &mapCallback);
  // pubPath = _nh.advertise<nav_msgs::Path>("path", 10);
  // char key = 'a';
  //
  // while(flag != 1 && key != 'q'){
  //   ros::spinOnce();
  //   std::cout << "flag : " << flag << std::endl;
  // }
  //
  // int radius = ceil(robotRadius / resolution);
  // std::cout << "Radius : " << radius << std::endl;
  // RRT_tree t;
  // // cv::Mat map = cv::imread("/home/sj/map.jpg");
  // // if(! map.data )                              // Check for invalid input
  // // {
  // //   std::cout <<  "Could not open or find the image" << std::endl ;
  // //   return -1;
  // // }
  // cv::cvtColor(map,map,CV_GRAY2BGR);
  //
  // cv::imshow("test", map);
  // cv::waitKey(0);
  // std::cout << "Starting rrt" << std::endl;
  // std::vector<RRT_node> path = t.findPath(50, 50, 550, 550, radius, map, true, 20);
  // std::cout << "rrt finished" << std::endl;
  // createPathMsg(path);
  //
  // while(_nh.ok()){
  //   pubPath.publish(pathMsg);
  //   ros::spinOnce();
  // }


  return 0;
}
