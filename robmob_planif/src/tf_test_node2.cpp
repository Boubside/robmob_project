#include <ros/ros.h>
#include "ros/console.h"
#include <iostream>
#include <tf/transform_listener.h>


void test_tf(){
    tf::TransformListener listener;
    bool gotit = false;
    float x,y;
    while (ros::ok() && !gotit){
        tf::StampedTransform transform;
        try{
            ros::Time now = ros::Time::now();
            listener.waitForTransform("base_footprint", "base_link",now, ros::Duration(2.0));
            listener.lookupTransform("base_footprint", "base_link", ros::Time(0), transform);
            gotit = true;
        }
        catch (tf::TransformException ex){
            ROS_INFO("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
    }
    std::cout<<"x:"<<x<<std::endl;
    std::cout<<"y:"<<y<<std::endl;   
}
    
/*** MAIN ***/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_tf");
    test_tf();

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        //waitKey(10);
    }
    //ros::spin();
    return 0;
}
