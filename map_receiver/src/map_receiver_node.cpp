#include "map_receiver/map_receiver_node.h"

map_receiver_node::map_receiver_node(){
  cout<<"start map_receiver \n"<<endl;
  flag = 0;
  occupancyGrid = _nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &map_receiver_node::mapCallback, this);
}

map_receiver_node::~map_receiver_node(){
  cout<<"end map_receiver"<<endl;
}


void map_receiver_node::showMap(){
}

void map_receiver_node::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid){
  if (flag==0){
    cout<<"Map received !"<<endl;
    flag = 1;
    map = Mat::zeros(grid->info.height,grid->info.width, CV_8UC1);
    vector<signed char> vect;
    vect = grid->data;

    for (int i = 0; i< map.rows; i++){
      for (int j = 0; j< map.cols; j++){
        if (vect[i*map.cols + j] == -1)
          map.at<uchar>(i,j) = 150;
        if (vect[i*map.cols + j] == 0)
          map.at<uchar>(i,j) = 255;
        if (vect[i*map.cols + j] == 100)
          map.at<uchar>(i,j) = 0;
      }
    }
    imshow("Map",map);
    waitKey(0);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "map_receiver_node");
  map_receiver_node map_receiver;
  while (map_receiver.getFlag()==0) ros::spinOnce();
  map_receiver.showMap();
}
