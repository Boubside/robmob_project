#include "load_map/load_map_node.h"

load_map_node::load_map_node(){
  path = "src/load_map/testMap/map.jpg";
  map_pub_ = _nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
  cout<<"start load_map"<<endl;
}

load_map_node::~load_map_node(){
  cout<<"end load_map"<<endl;
}

int load_map_node::getMap(){
  map = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
  if( map.empty() ){
    cout <<  "Could not open or find the image" << endl ;
    return 0;
  }
  return 1;
}

//----------------Create the Occupancy Map---------------------//
void load_map_node::processMap(){

  Mat wallMap, clearMap, unknownMap;

  // Binary Threshold
  threshold(map,wallMap, 1, 1, THRESH_BINARY_INV);
  threshold(map,clearMap, 250, 1, THRESH_BINARY);
  unknownMap = 1 - (wallMap + clearMap);
  procMap = 100*wallMap + 101 * unknownMap; //Occupancy Map

}

void load_map_node::sendMap(){
  grid.info.resolution = 0.05;
  grid.info.width = procMap.cols;
  grid.info.height = procMap.rows;

  //transform Occupancy Map to Occupancy Grid
  vector<signed char> data;
  for (int i = 0; i< procMap.rows; i++)
    for (int j = 0; j< procMap.cols; j++)
    if (procMap.at<uchar>(i,j) > 100)
      data.push_back(-1);
    else
      data.push_back(procMap.at<uchar>(i,j));

  grid.data = data;
  grid.info.origin.position.x = -100.000000;
  grid.info.origin.position.y = -100.000000;

  // for (int k : data)    //Print data
  //   cout << k << " ";

  map_pub_.publish(grid);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "load_map_node");
  load_map_node load_map;
  int flag = load_map.getMap();
  if(flag == 0)
    return 0;

  load_map.processMap();
  for(int i=0;i<3;i++){
    load_map.sendMap();
    ros::spinOnce();
    waitKey(1000);
  }



}
