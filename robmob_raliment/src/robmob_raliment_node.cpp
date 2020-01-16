#include <cmath>
#include "robmob_raliment/robmob_raliment_node.h"


void robmob_raliment_node::pathCallback(const nav_msgs::Path::ConstPtr& path_)
{
  if(!_pathReceived)
  {
    _pathReceived = true;

    geometry_msgs::PoseStamped p;
    for(size_t i = 0; i < path_->poses.size();i++)
    {
      p.pose.position.x = path_->poses[i].pose.position.x;
      p.pose.position.y = path_->poses[i].pose.position.y;
      _path.poses.push_back(p);
    }
  }
}



robmob_raliment_node::robmob_raliment_node(double l1, double k1, double k2, double reachedRadius):
_l1(l1), _k1(k1), _k2(k2), _reachedRadius(reachedRadius)
{
  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _path_sub = _nh.subscribe<nav_msgs::Path>("path", 10, &robmob_raliment_node::pathCallback, this);
}

robmob_raliment_node::robmob_raliment_node(const robmob_raliment_node& other):
_pathReceived(other._pathReceived), _l1(other._l1), _k1(other._k1), _k2(other._k2), _reachedRadius(other._reachedRadius)
{
  geometry_msgs::PoseStamped p;
  for(size_t i = 0; i < other._path.poses.size();i++)
  {
    p.pose.position.x = other._path.poses[i].pose.position.x;
    p.pose.position.y = other._path.poses[i].pose.position.y;
    _path.poses.push_back(p);
  }

  _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  _path_sub = _nh.subscribe<nav_msgs::Path>("path", 10, &robmob_raliment_node::pathCallback, this);
}




bool robmob_raliment_node::goalReached()
{
  return pointReached(_path.poses.size()-1);
}

bool robmob_raliment_node::currentPointReached()
{
  return pointReached(_inPathTargetIndex);
}

bool robmob_raliment_node::pointReached(size_t inPathIndex)
{
  if(inPathIndex > _path.poses.size()-1){
    std::cerr << "Error : Trying to compiute if robot reached an inexistant point" << std::endl;
    return false;
  }

  double theta = _robotPose.orientation.z;
  double xp =_robotPose.position.x + _l1 * cos(theta);
  double yp =_robotPose.position.y + _l1 * sin(theta);
  double xr =_path.poses[inPathIndex].pose.position.x;
  double yr =_path.poses[inPathIndex].pose.position.y;

  if(sqrt((xp-xr)*(xp-xr) + (yp-yr)*(yp-yr)) < _reachedRadius) return true;
  else return false;
}

void robmob_raliment_node::getRobotPose()
{
  double x, y, theta;

  tf::TransformListener listener;
  bool found = false;
  while(ros::ok() && !found){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(2.0));
      listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
      found = true;
    }
    catch(tf::TransformException e){
      ROS_INFO("%s", e.what());
      ros::Duration(1.0).sleep();
    }
    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    theta = yaw;
  }

  std::cout << x << ", " << y << " ," << theta <<   std::endl;

  _robotPose.position.x = x;
  _robotPose.position.y = y;
  _robotPose.orientation.z = theta;
}

void robmob_raliment_node::stopMovement()
{
  _command.linear.x = 0;
  _command.angular.z = 0;
  _vel_pub.publish(_command);
}





void robmob_raliment_node::generateCommand()
{
  double theta = _robotPose.orientation.z;

  double v1 = -_k1 * (_robotPose.position.x + _l1 * cos(theta) - _path.poses[_inPathTargetIndex].pose.position.x);
  double v2 = -_k2 * (_robotPose.position.y + _l1 * sin(theta) - _path.poses[_inPathTargetIndex].pose.position.y);

  double linearSpeed = cos(theta)*v1 + sin(theta)*v2;
  double rotationSpeed = _l1*(-sin(theta)*v1 + cos(theta)*v2);

  double ratio = fabs(linearSpeed / _maxLinearSpeed);
  if(ratio > 1)
  {
    linearSpeed /= ratio;
    std::cout << "add to cap the linear speed, ratio was " << ratio <<   std::endl;
  }

  ratio = fabs(rotationSpeed / _maxRotationSpeed);
  if(ratio > 1)
  {
    rotationSpeed /= ratio;
    std::cout << "add to cap the rotation speed, ratio was " << ratio << std::endl;
  }

  _command.linear.x = linearSpeed;
  _command.angular.z = rotationSpeed;
}


void robmob_raliment_node::run()
{
  // while(ros::ok() && !_pathReceived)
  //   ros::spinOnce();

  geometry_msgs::PoseStamped p;
  p.pose.position.x = 0;
  p.pose.position.y = 2;
  _path.poses.push_back(p);
  p.pose.position.x = -3;
  p.pose.position.y = 2;
  _path.poses.push_back(p);
  p.pose.position.x = -4;
  p.pose.position.y = -8;
  _path.poses.push_back(p);
  p.pose.position.x = -6;
  p.pose.position.y = -8;
  _path.poses.push_back(p);
  p.pose.position.x = -6;
  p.pose.position.y = 2;
  _path.poses.push_back(p);
  p.pose.position.x = -8;
  p.pose.position.y = 2;
  _path.poses.push_back(p);

  while(ros::ok() && !goalReached())
  {
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    getRobotPose();

    if(currentPointReached())
    {
      _inPathTargetIndex++;
      std::cout << "Point reached !" << std::endl;
    }

    generateCommand();
    std::cout << "Command : " << _command.linear.x << ", " << _command.angular.z << std::endl;
    std::cout << "Current index in path : " << _inPathTargetIndex << std::endl;
    std::cout << "Target point : " << _path.poses[_inPathTargetIndex].pose.position.x << " ," <<  _path.poses[_inPathTargetIndex].pose.position.y << std::endl;
    _vel_pub.publish(_command);
  }

  stopMovement();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rombmob_robmob_raliment_node_node");
  robmob_raliment_node raliment;
  raliment.run();

  return 0;
}
