#include "robmob_planif/RRT_tree.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(){

  RRT_tree t;
  cv::Mat map = cv::imread("/home/sj/map.jpg");
  cv::Mat path = map.clone();
  if(! map.data )                              // Check for invalid input
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  cv::namedWindow( "RRT tree", CV_WINDOW_AUTOSIZE );
  cv::namedWindow( "Path", CV_WINDOW_AUTOSIZE );

  t.buildTree(50, 50, 550, 550, map, 30);
  t.calculatePath(50, 50, 550, 550);
  t.drawTree(&map, 550, 550);
  t.drawPath(&path, 550, 550);

  cv::waitKey(0);

  return 0;
}
