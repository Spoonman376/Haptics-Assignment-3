
#include "OctTree.h"

OctTree::OctTree(vector<cVector3d> ps, cVector3d cp, double x, double y, double z)
{

  centerPoint = cp;
  distx = x;
  disty = y;
  distz = z;
  
  // if there are more than 100 points split
  if (ps.size() > 100) {
    
    vector<cVector3d> lll, llg, lgl, lgg, gll, glg, ggl, ggg;
    
    for (int i = 0; i < ps.size(); ++i) {
      cVector3d p = ps[i];
      
      if (p.x() < centerPoint.x() && p.y() < centerPoint.y() && p.z() < centerPoint.z()) {
        lll.push_back(p);
      }
      else if (p.x() < centerPoint.x() && p.y() < centerPoint.y() && p.z() > centerPoint.z()) {
        llg.push_back(p);
      }
      else if (p.x() < centerPoint.x() && p.y() > centerPoint.y() && p.z() < centerPoint.z()) {
        lgl.push_back(p);
      }
      else if (p.x() < centerPoint.x() && p.y() > centerPoint.y() && p.z() > centerPoint.z()) {
        lgg.push_back(p);
      }
      else if (p.x() > centerPoint.x() && p.y() < centerPoint.y() && p.z() < centerPoint.z()) {
        gll.push_back(p);
      }
      else if (p.x() > centerPoint.x() && p.y() < centerPoint.y() && p.z() > centerPoint.z()) {
        glg.push_back(p);
      }
      else if (p.x() > centerPoint.x() && p.y() > centerPoint.y() && p.z() < centerPoint.z()) {
        ggl.push_back(p);
      }
      else if (p.x() > centerPoint.x() && p.y() > centerPoint.y() && p.z() > centerPoint.z()) {
        ggg.push_back(p);
      }
      else  {
        points.push_back(p);
      }
    }
    
    
    children.push_back(new OctTree(lll, cp + cVector3d(-x/2, -y/2, -z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(llg, cp + cVector3d(-x/2, -y/2,  z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(lgl, cp + cVector3d(-x/2,  y/2, -z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(lgg, cp + cVector3d(-x/2,  y/2,  z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(gll, cp + cVector3d( x/2, -y/2, -z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(glg, cp + cVector3d( x/2, -y/2,  z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(ggl, cp + cVector3d( x/2,  y/2, -z/2), x/2, y/2, z/2));
    children.push_back(new OctTree(ggg, cp + cVector3d( x/2,  y/2,  z/2), x/2, y/2, z/2));

  }
  else {
    points = ps;
  }
  
}


vector<cVector3d> OctTree::getPointsForArea(cVector3d cp, double r)
{

  vector<cVector3d> pointsInArea; 
  if (abs(cp.x() - centerPoint.x()) < distx + r && abs(cp.y() - centerPoint.y()) < disty + r && abs(cp.z() - centerPoint.z()) < distz + r) 
  {
    for (cVector3d point : points)
    {
      if ((cp - point).length() <= r)
        pointsInArea.push_back(point);
    }
 
    for (OctTree* child : children)
    {
      vector<cVector3d> childPoints = child->getPointsForArea(cp, r);
      copy(childPoints.begin(), childPoints.end(), back_inserter(pointsInArea));
    }
  }

  return pointsInArea;
}
