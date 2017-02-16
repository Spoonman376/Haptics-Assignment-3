
#include "SphereTree.h"

SphereTree::SphereTree(vector<cVector3d> & ps)
{
  points = ps;

  // find the smallest sphere encapsulating the points.
  cVector3d cp = (points[0] + points[1]) / 2;
  double r = (points[0] - points[1]).length() / 2;

  for (int i = 2; i < points.size(); ++i)
  {

    cVector3d p = points[i];
    if ((cp - p).length() > r)
    {
      cVector3d n = cp - p;
      n.normalize();

      cp = ((cp + n * r) + p) / 2;
      r = (cp - p).length();
    }

    if (points.size() > 100)
    {
      vector<vector<cVector3d>> childPoints;
    }
  }

  centerPoint = cp;
  radius = r;



}

SphereTree::~SphereTree()
{
}
