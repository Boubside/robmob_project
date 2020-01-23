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
  _twist.angular.z = a_scale*joy->axes[angular_];
  _twist.linear.x = l_scale*joy->axes[linear_];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robmob_teleop_node");
  robmob_teleop_node teleop;
  ros::Rate rate(100);

  while(ros::ok()){
    rate.sleep();
    ros::spinOnce();
    teleop.vel_pub_.publish(teleop._twist);
  }
}
