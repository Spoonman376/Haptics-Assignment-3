#ifndef OCTTREE_H
#define OCTTREE_H

#include "chai3d.h"
#include <vector>
#include <algorithm>
#include <iterator>

using namespace chai3d;
using namespace std;

class OctTree
{

  cVector3d centerPoint;
  double distx, disty, distz;

  vector<OctTree*> children;

  vector<cVector3d> points;

public:
  
  OctTree(vector<cVector3d> ps, cVector3d cp, double x, double y, double z);
  ~OctTree();

  // Takes a centerpoint and a radius. Will treat the cursor as a cube
  vector<cVector3d> getPointsForArea(cVector3d, double);



};



#endif
