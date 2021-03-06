#ifndef ROBMOBTELEOP_H
#define ROBMOBTELEOP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class robmob_teleop_node{

private:

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle _nh;

  int linear_, angular_;
  double l_scale = 1;
  double a_scale = 1;
  ros::Subscriber joy_sub_;

public:
  ros::Publisher vel_pub_;
  geometry_msgs::Twist _twist;
  bool _pause = false;

  robmob_teleop_node();
  ~robmob_teleop_node();
};

#endif
