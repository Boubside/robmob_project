#include "robmob_teleop/robmob_teleop_node.h"

robmob_teleop_node::robmob_teleop_node(){
  linear_ = 1;
  angular_ = 0;
  vel_pub_ = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &robmob_teleop_node::joyCallback, this);
}

robmob_teleop_node::~robmob_teleop_node(){

}

void robmob_teleop_node::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*joy->axes[angular_];
  twist.linear.x = l_scale*joy->axes[linear_];
  vel_pub_.publish(twist);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robmob_teleop_node");
  robmob_teleop_node teleop;
 
  ros::spin();
}
