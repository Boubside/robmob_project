#include "robmob_planif/RRT_tree.hpp"
#include <time.h>
#include <ros/ros.h>


RRT_node::RRT_node (){
  _x = _y = 0;
  _xp = _yp = -1;
}

RRT_node::RRT_node (int x, int y){
  _x = x;
  _y = y;
  _xp = _yp = -1;
}

RRT_node::RRT_node (int x, int y, int xp, int yp){
  _x = x;
  _y = y;
  _xp = xp;
  _yp = yp;
}

RRT_node::RRT_node (const RRT_node& other){
  _x = other.getX();
  _y = other.getY();
  _xp = other.getXp();
  _yp = other.getYp();
}

std::string RRT_node::toString(){
  if(hasParent()) return "[x=" + std::to_string(_x) + ",y=" + std::to_string(_y) + ",parent=" + std::to_string(_xp) + "," + std::to_string(_yp) + "]";
  else return "[x=" + std::to_string(_x) + ",y=" + std::to_string(_y) + "]";
}

RRT_node RRT_node::operator=(const RRT_node other){
  _x = other.getX();
  _y = other.getY();
  _xp = other.getXp();
  _yp = other.getYp();
}

RRT_node::~RRT_node(){

}



RRT_tree::RRT_tree (){
  srand(time(NULL));
}

RRT_tree::~RRT_tree (){

}

//Add a new leaf to the RRt tree
void RRT_tree::addLeaf(const RRT_node& leaf){
  _tree.push_back(leaf);
}

void RRT_tree::addLeaf(int x, int y, int xp, int yp){
  _tree.push_back(RRT_node(x, y, xp, yp));
}

//Draw the tree on map and print to screen
void RRT_tree::drawTree(cv::Mat* map, int xg, int yg, std::string windowName){
  for(int i = 0; i < _tree.size(); i++){
    if(i == 0) cv::circle(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), 3, cv::Scalar(0,255,0),3);
    else{
      cv::circle(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), 3, cv::Scalar(255,0,0), 3);
      cv::line(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), cv::Point(_tree[i].getXp(),_tree[i].getYp()), cv::Scalar(100,100,100), 3);
    }
  }
  cv::circle(*map, cv::Point(xg,yg), 3, cv::Scalar(0,0,255), 3);
  imshow(windowName, *map);
}

//Draw the path
void RRT_tree::drawPath(cv::Mat* map, int xg, int yg, std::string windowName){
  for(int i = 0; i < _path.size(); i++){
    cv::circle(*map, cv::Point(_path[i].getX(),_path[i].getY()), 3, cv::Scalar(255,0,0), 3);
    if(_path[i].hasParent()) cv::line(*map, cv::Point(_path[i].getX(),_path[i].getY()), cv::Point(_path[i].getXp(),_path[i].getYp()), cv::Scalar(100,100,100), 3);
  }
  cv::circle(*map, cv::Point(_path[0].getX(),_path[0].getY()), 3, cv::Scalar(0,255,0),3);
  cv::circle(*map, cv::Point(xg,yg), 3, cv::Scalar(0,0,255), 3);
  imshow(windowName, *map);
}

//Build the RRT tree
void RRT_tree::buildTree(int xi, int yi, int xg, int yg, cv::Mat map, int dq, int maxIterations){
  //Adding initial node
  addLeaf(xi, yi);
  int xnew = xi;
  int ynew = yi;
  int xr, yr, xnear, ynear;
  int itCount = 0;

  //Converting image to grayscale for treatement
  cv::Mat gray;
  cv::cvtColor(map, gray, CV_BGR2GRAY);

  //RRT algorithm
  while(!goalReached(xg ,yg, xnew, ynew, dq) && itCount++ < maxIterations && ros::ok()){
    // std::cout << "Building tree, iteration : " << itCount << std::endl;
    randFreeConf(&xr, &yr, gray);
    nearestNode(xr, yr, &xnear, &ynear);
    if(newConfig(xnear, ynear, xr, yr, &xnew, &ynew, dq, gray)){
      addLeaf(xnew, ynew, xnear, ynear);
    }
  }
  addLeaf(xg, yg, xnew, ynew);
}

bool RRT_tree::goalReached(int xg, int yg, int xnew, int ynew, int dq){
  return (sqrt((xg-xnew)*(xg-xnew) + (yg-ynew)*(yg-ynew)) < dq);
}

void RRT_tree::randFreeConf(int* xr, int* yr, cv::Mat map){
  double x, y;
  do{
    x = map.cols * (double) rand()/RAND_MAX;
    y = map.rows * (double) rand()/RAND_MAX;
  }while(map.at<uchar>(y, x) != 255);
  *xr = x;
  *yr = y;
}

void RRT_tree::nearestNode(int xr, int yr, int* xnear, int *ynear){
  int argmin = 0;
  double min = sqrt((xr-_tree[0].getX())*(xr-_tree[0].getX()) + (yr-_tree[0].getY())*(yr-_tree[0].getY()));
  double dist;
  for(unsigned int i = 1; i < _tree.size(); i++){
    dist = sqrt((xr-_tree[i].getX())*(xr-_tree[i].getX()) + (yr-_tree[i].getY())*(yr-_tree[i].getY()));
    if(dist < min){
      min = dist;
      argmin = i;
    }
  }
  *xnear = _tree[argmin].getX();
  *ynear = _tree[argmin].getY();
}

bool RRT_tree::newConfig(int xnear, int ynear, int xr, int yr, int *xnew, int *ynew, int dq, cv::Mat map){
  int x, y;
  double d = sqrt((yr - ynear)*(yr - ynear) + (xr - xnear)*(xr - xnear));

  x = xnear + ((xr-xnear)*(d/dq));
  y = ynear + ((yr-ynear)*(d/dq));

  cv::LineIterator it(map, cv::Point(xnear, ynear), cv::Point(x, y));

  for(int i = 0; i < it.count; i++, ++it)
  {
    if(map.at<uchar>(it.pos()) == 0) return false;
  }

  *xnew = x;
  *ynew = y;
  return true;
}


void RRT_tree::inflateObstacles(cv::Mat *map, int radius){
  cv::Mat dilatation_dst;
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius+1, 2*radius+1), cv::Point(radius, radius));
  erode(*map, dilatation_dst, element );
  *map = dilatation_dst;
}


RRT_node& RRT_tree::findParent(int xp, int yp){
  for(RRT_node& n : _tree){
    if(n.getX() == xp && n.getY() == yp){
      return n;
    }
  }
}

void RRT_tree::calculatePath(int xi, int yi, int xg, int yg){
  RRT_node cur;
  auto it = _path.insert(_path.begin(), _tree[_tree.size()-1]);
  it = _path.begin();
  while(_path[0].getX() != xi || _path[0].getY() != yi){
    if(_path[0].hasParent()) _path.insert(it, findParent(_path[0].getXp(), _path[0].getYp()));
    it = _path.begin();
  }
}



std::string RRT_tree::toString(){
  std::string str = "RRT tree of size " + std::to_string(_tree.size()) + " :\n";
  for(int i = 0; i < _tree.size(); i++){
    str += "\tNode #" + std::to_string(i) + " : " + _tree[i].toString() + "\n";
  }
  return str;
}


void RRT_tree::removeUnecessaryNodes(cv::Mat map, int xg, int yg){
  std::vector<RRT_node> newPath;
  bool clear;
  int lastVisiblePoint = 0;
  newPath.push_back(_path[0]);

  cv::Mat gray;
  cv::cvtColor(map, gray, CV_BGR2GRAY);

  for(int i = 0; i < newPath.size(); i++){
    if(newPath[i].getX() != xg && newPath[i].getX() != yg){
      for(int j = lastVisiblePoint+1; j < _path.size(); j++){
        clear = true;
        // lastVisiblePoint = j;

        cv::LineIterator it(gray, cv::Point(newPath[i].getX(), newPath[i].getY()), cv::Point(_path[j].getX(), _path[j].getY()));

        //Checking if path from i to j is free
        for(int i = 0; i < it.count; i++, ++it){
          if(gray.at<uchar>(it.pos()) == 0){
            clear = false;
            break;
          }
        }

        if(clear){
          lastVisiblePoint = j;
        }
      }
      newPath.push_back(_path[lastVisiblePoint]);
      newPath[i+1].setParent(newPath[i].getX(), newPath[i].getY());
    }
  }

  _path = newPath;
}


std::vector<RRT_node> RRT_tree::findPath(int xi, int yi, int xg, int yg, int robotRadius, cv::Mat map, bool draw, int dq, int maxIterations){
  inflateObstacles(&map, robotRadius);
  cv::Mat path = map.clone();
  cv::Mat newPath = map.clone();


  buildTree(xi, yi, xg, yg, map, dq, maxIterations);
  calculatePath(xi, yi, xg, yg);
  // if(draw) drawTree(&map, xg, yg, "RRT tree");
  // if(draw) drawPath(&path, xg, yg, "Raw path");

  removeUnecessaryNodes(map, xg, yg);
  if(draw) drawPath(&newPath, xg, yg, "Smoothed path");

  return _path;
}
