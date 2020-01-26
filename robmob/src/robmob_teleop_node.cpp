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
  ros::Rate rate(2);
  _twist.angular.z = a_scale*joy->axes[angular_];
  _twist.linear.x = l_scale*joy->axes[linear_];
  if(joy->buttons[3] == 1 && !_pause)
  {
    _pause = true;
    rate.sleep();
  }
  else if(joy->buttons[3] == 1 && _pause)
  {
    _pause = false;
    rate.sleep();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robmob_teleop_node");
  robmob_teleop_node teleop;
  ros::Rate rate(100);

  while(ros::ok())
  {
    if(!teleop._pause){
      teleop.vel_pub_.publish(teleop._twist);
    }
    rate.sleep();
    ros::spinOnce();
  }
}
