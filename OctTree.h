#ifndef OCTTREE_H
#define OCTTREE_H

#include "chai3d.h"
#include <vector>
#include <algorithm>
#include <iterator>

using namespace chai3d;
using namespace std;

struct ColoredPoint
{
  cVector3d point;
  int colorIndex;

  ColoredPoint(cVector3d p, int cI)
  {
    point = p;
    colorIndex = cI;
  }
};

class OctTree
{

  cVector3d centerPoint;
  double distx, disty, distz;

  vector<OctTree*> children;

  vector<ColoredPoint> points;

public:
  
  OctTree(vector<cVector3d> ps, vector<int> cIs, cVector3d cp, double x, double y, double z);
  ~OctTree();

  // Takes a centerpoint and a radius. Will treat the cursor as a cube
  void getPointsForArea(vector<cVector3d>&, cVector3d, double);
  vector<int> getCIsForArea(cVector3d, double);



};



#endif
