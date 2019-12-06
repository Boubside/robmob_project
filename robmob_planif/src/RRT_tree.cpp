#include "robmob_planif/RRT_tree.hpp"
#include <time.h>


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

void RRT_tree::addLeaf(const RRT_node& leaf){
  _tree.push_back(leaf);
}

void RRT_tree::addLeaf(int x, int y, int xp, int yp){
  _tree.push_back(RRT_node(x, y, xp, yp));
}

void RRT_tree::drawTree(cv::Mat* map, int xg, int yg){
  for(int i = 0; i < _tree.size(); i++){
    if(i == 0) cv::circle(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), 3, cv::Scalar(0,255,0),3);
    else{
      cv::circle(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), 3, cv::Scalar(255,0,0), 3);
      cv::line(*map, cv::Point(_tree[i].getX(),_tree[i].getY()), cv::Point(_tree[i].getXp(),_tree[i].getYp()), cv::Scalar(100,100,100), 3);
    }
  }
  cv::circle(*map, cv::Point(xg,yg), 3, cv::Scalar(0,0,255), 3);
}

void RRT_tree::drawPath(cv::Mat* map, int xg, int yg){
  for(int i = 0; i < _path.size(); i++){
    if(i == 0) cv::circle(*map, cv::Point(_path[i].getX(),_path[i].getY()), 3, cv::Scalar(0,255,0),3);
    else{
      cv::circle(*map, cv::Point(_path[i].getX(),_path[i].getY()), 3, cv::Scalar(255,0,0), 3);
      cv::line(*map, cv::Point(_path[i].getX(),_path[i].getY()), cv::Point(_path[i].getXp(),_path[i].getYp()), cv::Scalar(100,100,100), 3);
    }
  }
  cv::circle(*map, cv::Point(xg,yg), 3, cv::Scalar(0,0,255), 3);
}

void RRT_tree::buildTree(int xi, int yi, int xg, int yg, cv::Mat map, int dq, int maxIterations){
  addLeaf(xi, yi);
  int xnew = xi;
  int ynew = yi;
  int xr, yr, xnear, ynear;
  int itCount = 0;

  cv::Mat gray;
  cv::cvtColor(map, gray, CV_BGR2GRAY);

  while(!goalReached(xg ,yg, xnew, ynew, dq) && itCount++ < maxIterations){
    randFreeConf(&xr, &yr, gray);
    nearestNode(xr, yr, &xnear, &ynear);
    if(newConfig(xnear, ynear, xr, yr, &xnew, &ynew, dq, gray)){
      addLeaf(xnew, ynew, xnear, ynear);
    }
    // drawTree(&map, xg, yg);
    imshow("test", map);
  }
  addLeaf(xg, yg, xnew, ynew);
  std::cout << toString() << std::endl;
}

bool RRT_tree::goalReached(int xg, int yg, int xnew, int ynew, int dq){
  return (sqrt((xg-xnew)*(xg-xnew) + (yg-ynew)*(yg-ynew)) < dq);
}

void RRT_tree::randFreeConf(int* xr, int* yr, cv::Mat map){
  double x, y;
  do{
    x = map.cols * (double) rand()/RAND_MAX;
    y = map.rows * (double) rand()/RAND_MAX;
  }while(map.at<uchar>(y, x) == 0);
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


RRT_node& RRT_tree::findParent(int xp, int yp){
  for(RRT_node& n : _tree){
    if(n.getX() == xp && n.getY() == yp){
      return n;
    }
  }
  return children;
}

void RRT_tree::calculatePath(int xi, int yi, int xg, int yg){
  RRT_node& cur;
  auto it = _path.insert(vec.begin(), _tree[_tree.size()-1]);
  while(_path[0].getXp() != xi || _path[0].getYp() != yi){
    _path.insert(it, findParent(_path[0].getXp(), _path[0].getYp()));
  }
}



std::string RRT_tree::toString(){
  std::string str = "RRT tree of size " + std::to_string(_tree.size()) + " :\n";
  for(int i = 0; i < _tree.size(); i++){
    str += "\tNode #" + std::to_string(i) + " : " + _tree[i].toString() + "\n";
  }
  return str;
}
