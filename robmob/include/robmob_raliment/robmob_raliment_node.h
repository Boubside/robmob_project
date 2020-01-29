#ifndef RALIMENT_H
#define RALIMENT_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>

class robmob_raliment_node{

  private:
    ros::NodeHandle _nh;

    bool _pathReceived = false;
    nav_msgs::Path _path;

    size_t _inPathTargetIndex = 0;
    double _l1 = 0.3;
    double _k1 = 2;
    double _k2 = 2;
    double _maxLinearSpeed = 0.5;
    double _maxRotationSpeed = 1;
    double _reachedRadius;

    geometry_msgs::Pose _robotPose;
    geometry_msgs::Twist _command;

    ros::Publisher _vel_pub;
    ros::Subscriber _path_sub;


    void pathCallback(const nav_msgs::Path::ConstPtr& path_);

    bool goalReached();
    bool currentPointReached();
    bool pointReached(size_t inPathIndex);
    void getRobotPose();
    void stopMovement();

  public:

    robmob_raliment_node(double l1 = 0.2, double k1 = 10, double k2 = 0.5, double reachedRadius = 0.5);
    robmob_raliment_node(const robmob_raliment_node& other);

    nav_msgs::Path path() const { return _path; }
    geometry_msgs::Twist command() const { return _command; }

    void generateCommand();
    void run();



};

#endif
