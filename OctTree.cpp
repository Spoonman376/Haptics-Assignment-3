
#include "OctTree.h"

OctTree::OctTree(vector<cVector3d> ps, vector<int> cIs, cVector3d cp, double x, double y, double z)
{

  centerPoint = cp;
  distx = x;
  disty = y;
  distz = z;
  
  // if there are more than 25 points split
  if (ps.size() > 25) {
    
    vector<cVector3d> lll, llg, lgl, lgg, gll, glg, ggl, ggg;
    vector<int> clll, cllg, clgl, clgg, cgll, cglg, cggl, cggg;

    for (int i = 0; i < ps.size(); ++i) {
      cVector3d p = ps[i];
      int c = cIs[i];

      if (p.x() < centerPoint.x() && p.y() < centerPoint.y() && p.z() < centerPoint.z()) {
        lll.push_back(p);
        clll.push_back(c);
      }
      else if (p.x() < centerPoint.x() && p.y() < centerPoint.y() && p.z() > centerPoint.z()) {
        llg.push_back(p);
        cllg.push_back(c);
      }
      else if (p.x() < centerPoint.x() && p.y() > centerPoint.y() && p.z() < centerPoint.z()) {
        lgl.push_back(p);
        clgl.push_back(c);
      }
      else if (p.x() < centerPoint.x() && p.y() > centerPoint.y() && p.z() > centerPoint.z()) {
        lgg.push_back(p);
        clgg.push_back(c);
      }
      else if (p.x() > centerPoint.x() && p.y() < centerPoint.y() && p.z() < centerPoint.z()) {
        gll.push_back(p);
        cgll.push_back(c);
      }
      else if (p.x() > centerPoint.x() && p.y() < centerPoint.y() && p.z() > centerPoint.z()) {
        glg.push_back(p);
        cglg.push_back(c);
      }
      else if (p.x() > centerPoint.x() && p.y() > centerPoint.y() && p.z() < centerPoint.z()) {
        ggl.push_back(p);
        cggl.push_back(c);
      }
      else if (p.x() > centerPoint.x() && p.y() > centerPoint.y() && p.z() > centerPoint.z()) {
        ggg.push_back(p);
        cggg.push_back(c);
      }
      else  {
        points.push_back(ColoredPoint(p, c));
      }
    }
    
    children.push_back(new OctTree(lll, clll, cp + cVector3d(-x / 2, -y / 2, -z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(llg, cllg, cp + cVector3d(-x / 2, -y / 2, z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(lgl, clgl, cp + cVector3d(-x / 2, y / 2, -z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(lgg, clgg, cp + cVector3d(-x / 2, y / 2, z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(gll, cgll, cp + cVector3d(x / 2, -y / 2, -z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(glg, cglg, cp + cVector3d(x / 2, -y / 2, z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(ggl, cggl, cp + cVector3d(x / 2, y / 2, -z / 2), x / 2, y / 2, z / 2));
    children.push_back(new OctTree(ggg, cggg, cp + cVector3d( x/2,  y/2,  z/2), x/2, y/2, z/2));

  }
  else {
    for (int i = 0; i < ps.size(); ++i)
    {
      points.push_back(ColoredPoint(ps[i], cIs[i]));
    }
  }
  
}


void OctTree::getPointsForArea(vector<cVector3d> &pointsInArea, cVector3d cp, double r)
{

  if (abs(cp.x() - centerPoint.x()) < distx + r && abs(cp.y() - centerPoint.y()) < disty + r && abs(cp.z() - centerPoint.z()) < distz + r)
  {
    for (ColoredPoint cPoint : points)
    {
      if ((cp - cPoint.point).length() <= r)
        pointsInArea.push_back(cPoint.point);
    }
 
    for (OctTree* child : children)
    {
      child->getPointsForArea(pointsInArea, cp, r);
    }
  }
}

vector<int> OctTree::getCIsForArea(cVector3d cp, double r)
{

  vector<int> cIsInArea;
  if (abs(cp.x() - centerPoint.x()) < distx + r && abs(cp.y() - centerPoint.y()) < disty + r && abs(cp.z() - centerPoint.z()) < distz + r)
  {
    for (ColoredPoint cPoint : points)
    {
      if ((cp - cPoint.point).length() <= r)
        cIsInArea.push_back(cPoint.colorIndex);
    }

    for (OctTree* child : children)
    {
      vector<int> childcolorIndices = child->getCIsForArea(cp, r);
      copy(childcolorIndices.begin(), childcolorIndices.end(), back_inserter(cIsInArea));
    }
  }

  return cIsInArea;
}
