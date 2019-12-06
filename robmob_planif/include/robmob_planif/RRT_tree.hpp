#ifndef RRT_TREE_H
#define RRT_TREE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class RRT_node {

private:
  int _x;
  int _y;

  int _xp;
  int _yp;

public:
  RRT_node ();
  RRT_node (int x, int y);
  RRT_node (int x, int y, int xp, int yp);
  RRT_node (const RRT_node& other);
  RRT_node operator=(const RRT_node other);

  double getX() const{ return _x; }
  double getY() const{ return _y; }
  double getXp() const{ return _xp; }
  double getYp() const{ return _yp; }


  bool hasParent(){ return (_xp != -1 && _yp != -1); }

  std::string toString();


  ~RRT_node ();
};


class RRT_tree {
private:
  std::vector<RRT_node> _tree;
  std::vector<RRT_node> _path;

  bool goalReached(int xg, int yg, int xnew, int ynew, int dq);
  void randFreeConf(int* xr, int* yr, cv::Mat map);
  void nearestNode(int xr, int yr, int* xnear, int *ynear);
  bool newConfig(int xnear, int ynear, int xr, int yr, int *xnew, int *ynew, int dq, cv::Mat map);

public:
  RRT_tree ();
  ~RRT_tree ();

  void addLeaf(const RRT_node& leaf);
  void addLeaf(int x, int y, int xp = -1, int yp = -1);

  void buildTree(int xi, int yi, int xg, int yg, cv::Mat map, int dq = 30, int maxIterations = 100000);
  void drawTree(cv::Mat* map, int xg, int yg);
  void drawPath(cv::Mat* map, int xg, int yg);

  RRT_node& findParent(int xp, int yp);
  void calculatePath(int xi, int yi, int xg, int yg);

  std::string toString();
};

#endif
