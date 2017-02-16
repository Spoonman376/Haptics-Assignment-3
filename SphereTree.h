#ifndef SPHERETREE_H
#define SPHERETREE_H

#include "chai3d.h"
#include <vector>

using namespace chai3d;
using namespace std;

class SphereTree
{

public:
  
  SphereTree(vector<cVector3d> & p);
  ~SphereTree();

  cVector3d centerPoint;
  double radius;
  vector<cVector3d> points;

  vector<SphereTree*> children;



};



#endif