#include "map_receiver/map_receiver_node.h"

map_receiver_node::map_receiver_node(){
  cout<<"start map_receiver \n"<<endl;
  flag = 0;
  occupancyGrid = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &map_receiver_node::mapCallback, this);
}

map_receiver_node::~map_receiver_node(){
  cout<<"end map_receiver"<<endl;
}


void map_receiver_node::getMap(){
}

void map_receiver_node::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid){
  if (flag==0){
    cout<<"Map received !"<<endl;
    flag = 1;
    map = new Mat(grid->info.height,grid->info.width, CV_8UC1,Scalar(0));
    cout<<grid->info.height<<" "<<grid->info.width<<endl;
    cout<<map.rows<<" "<<map.cols<<endl;

  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_receiver_node");
  map_receiver_node map_receiver;
  while (map_receiver.getFlag()==0) ros::spinOnce();
  map_receiver.getMap();
}
