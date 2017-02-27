//===========================================================================
/*
    CPSC 599.86 / 601.86 - Computer Haptics
    Winter 2017, University of Calgary
 
    This class encapsulates the visual and haptic rendering for an
    unstructured point set. It inherits the regular CHAI3D cMultiMesh to
    enable loading of the point data as vertices from a .obj file.

    Your job is to implement the computeLocalInteraction() method to render
    the point set as an implicit surface by computing the proxy position
    from the tool position at every frame, subject to constraints defined
    by the point set.

    \author    <your name>
    \date      February 2017
*/
//===========================================================================

#ifndef POINTSET_H
#define POINTSET_H

#include "chai3d.h"
#include "OctTree.h"
#include "stdio.h"

class PointSet : public chai3d::cMultiMesh
{
    //! A visible sphere that tracks the position of the proxy on the surface
    chai3d::cShapeSphere m_projectedSphere;
    
    chai3d::cMesh m_tangentDisk;
    
    //! The points in the set.
    std::vector<chai3d::cVector3d> m_points;
    
    //! Colors associated with the points (if any).
    std::vector<chai3d::cColorf> m_colors;

    //! Helper function that computes the direction of minimal co-variance of
    //! a set of (relative) position vectors and corresponding weights.
    chai3d::cVector3d minimizeCovariance(const std::vector<chai3d::cVector3d> &a_positions,
                                         const std::vector<double> &a_weights);
    
    double surfaceFunction(cVector3d);
    cVector3d closestPointToPlane(cVector3d);
    
    cVector3d oldNormal = chai3d::cVector3d(0,0,0);
    cVector3d currentNormal = chai3d::cVector3d(0, 0, 0);
    cVector3d averagePoint = chai3d::cVector3d(0, 0, 0);
    bool moving = false;

    vector<cVector3d> oldLocalPoints;
    vector<cVector3d> localPoints;
    vector<int> localColorIndices;


    void setLocalPoints(cVector3d);

    void computeLocalInterationInside(const chai3d::cVector3d& a_toolPos,
                                      const chai3d::cVector3d& a_toolVel,
                                      const unsigned int a_IDN);


public:
    PointSet();
    virtual ~PointSet();

    OctTree* tree;
    double surfaceValue = 0;
    double radiusOfInfluence;


    //! Loads a point set from a ply, obj, or 3ds file.
    virtual bool loadFromFile(std::string a_filename);

    //! Contains code for graphically rendering this object in OpenGL.
    virtual void render(chai3d::cRenderOptions& a_options);

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const chai3d::cVector3d& a_toolPos,
                                         const chai3d::cVector3d& a_toolVel,
                                         const unsigned int a_IDN);
};

#endif
