#include "robmob_planif/RRT_tree.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(){

  RRT_tree t;
  cv::Mat map = cv::imread("/home/sj/map.jpg");
  if(! map.data )                              // Check for invalid input
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  cv::namedWindow( "test", CV_WINDOW_AUTOSIZE );
  imshow("test", map);
  t.buildTree(50, 50, 550, 550, map, 30);
  t.drawPath(&map, 550, 550);

  cv::waitKey(0);
 
  return 0;
}
