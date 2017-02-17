#ifndef OCTTREE_H
#define OCTTREE_H

#include "chai3d.h"
#include <vector>

using namespace chai3d;
using namespace std;

class OctTree
{

public:
  
  OctTree(vector<cVector3d> ps, cVector3d cp, double x, double y, double z);
  ~OctTree();

  cVector3d centerPoint;
  double distx, disty, distz;
  vector<cVector3d> points;

  vector<OctTree*> children;



};



#endif
