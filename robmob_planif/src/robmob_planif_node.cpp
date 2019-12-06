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

  std::vector<RRT_node> path = t.findPath(50, 50, 550, 550, map, true);

  cv::waitKey(0);

  return 0;
}
