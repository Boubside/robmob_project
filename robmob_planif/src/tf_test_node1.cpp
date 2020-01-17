#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iostream>

int main(int argc, char** argv){
std::cout << "test" << std::endl;
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
std::cout << "test" << std::endl;
      listener.waitForTransform("/base_link","/map",ros::Time::now(), ros::Duration(2.0));
std::cout << "test" << std::endl;
      listener.lookupTransform("/base_link", "/map",ros::Time(0), transform);
	std::cout << "test" << std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    std::cout << "x: " << transform.getOrigin().x() << std::endl;
    std::cout << "y: " << transform.getOrigin().y() << std::endl;

    rate.sleep();
  }
  return 0;
};
