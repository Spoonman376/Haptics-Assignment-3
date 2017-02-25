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

#include "PointSet.h"

#include <fstream>
#include <sstream>



using namespace chai3d;


PointSet::PointSet()
    : m_projectedSphere(0.05)
{
    // because we are haptically rendering this object as an implicit surface
    // rather than a set of polygons, we will not need a collision detector
    setCollisionDetector(0);
    
    // create a sphere that tracks the position of the proxy
    m_projectedSphere.m_material->setWhite();
    addChild(&m_projectedSphere);
    
    // create is disk that shows the tangent plane
    cCreateDisk(&m_tangentDisk, 0.2, 0.2);
    m_tangentDisk.setUseTransparency(true);
    m_tangentDisk.m_material->setTransparencyLevel(0.25);
    m_tangentDisk.m_material->m_specular.set(0, 0, 0, 1);
    m_projectedSphere.addChild(&m_tangentDisk);
}

PointSet::~PointSet()
{
    // remove the proxy tracking sphere so that the base class doesn't delete it
    m_projectedSphere.removeChild(&m_tangentDisk);
    removeChild(&m_projectedSphere);

    m_interactionInside = false;
}

//! Contains code to load a point set with color from a PLY file
bool PointSet::loadFromFile(std::string a_filename)
{
    using namespace std;
    bool result = false;

  double minX, minY, minZ =  10000;
  double maxX, maxY, maxZ = -10000;
  
    // detect PLY files to load here
    string extension = cGetFileExtension(a_filename);
  
  
    if (!extension.empty() && cStrToLower(extension) == "ply")
    {
        ifstream in(a_filename);
        if (!in.good())
        {
            cerr << "ERROR: Could not open file " << a_filename
                 << " for input!" << endl;
            return false;
        }
        
        string line, command, token;
        int nv = 0;
        
        // parse PLY header
        while (getline(in, line))
        {
            istringstream iss(line);
            iss >> command;
            if (command == "element") {
                iss >> token;
                if (token == "vertex") iss >> nv;
            }
            else if (command == "end_header")
                break;
        }
        
        // parse vertex data into a mesh structure
        cMesh *points = 0;
        if (nv) {
            points = newMesh();
            points->setUseVertexColors(true);
        }
        for (int i = 0; i < nv; ++i)
        {
            double x, y, z, r, g, b;
            getline(in, line);
            istringstream iss(line);
            iss >> x >> y >> z >> r >> g >> b;
            points->newVertex(cVector3d(x, y, z));
            points->m_vertices->setColor(i, r/255.f, g/255.f, b/255.f);
          
          maxX = max(x, maxX);
          maxY = max(y, maxY);
          maxZ = max(z, maxZ);

          minX = min(x, minX);
          minY = min(y, minY);
          minZ = min(z, minZ);
        }
        
        in.close();
        result = true;
    }
    // if not PLY file, use base class loader
    else result = cMultiMesh::loadFromFile(a_filename);
  
  
  
    // grab the points from the mesh(es) loaded
    if (result)
    {
        for (vector<cMesh*>::iterator it = m_meshes->begin();
             it != m_meshes->end(); ++it)
        {
            m_points = (*it)->m_vertices->m_localPos;
            m_colors = (*it)->m_vertices->m_color;
        }
        cout << "Loaded point set " << a_filename
             << " with " << m_points.size()
             << " elements." << endl;
    }
  
    vector<int> cI;
    for (int i = 0; i < m_colors.size(); ++i)
      cI.push_back(i);
      

  cVector3d cp = cVector3d((minX + maxX)/2, (minY + maxY)/2, (minY + maxY)/2);
  tree = new OctTree(m_points, cI, cp, (maxX - minX)/2, (maxY - minY)/2, (maxZ - minZ)/2);
  
    return result;
}

//! Contains code for graphically rendering this object in OpenGL.
void PointSet::render(cRenderOptions& a_options)
{
    // update the position and visibility of the proxy sphere
    m_projectedSphere.setShowEnabled(m_interactionInside);
    m_projectedSphere.setLocalPos(m_interactionPoint);
    
    // udpate the orientation of the tangent plane
    if (m_interactionInside && m_interactionNormal.lengthsq() > 0.0)
    {
        cVector3d z(0, 0, 1);
        cVector3d axis = cCross(z, m_interactionNormal);
        double angle = acos(cDot(z, m_interactionNormal));
        cMatrix3d rotate = cRotAxisAngleRad(axis.x(), axis.y(), axis.z(), angle);
        m_tangentDisk.setLocalRot(rotate);
    }
    
    if (SECTION_RENDER_OPAQUE_PARTS_ONLY(a_options))
    {
        // get previous OpenGL state to restore
        GLboolean lightingOn = glIsEnabled(GL_LIGHTING);
        GLboolean blendOn = glIsEnabled(GL_BLEND);
        GLfloat pointSize = 1.f;
        glGetFloatv(GL_POINT_SIZE, &pointSize);
        
        GLfloat viewport[4];
        glGetFloatv(GL_VIEWPORT, viewport);
        
        // render the point set
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glEnable(GL_POINT_SMOOTH);
        
        // try to determine an appropriate point size
        float ps = (0.125f * viewport[3]) / powf(m_points.size(), 1.f/3.f);
        if (ps < 1.f) ps = 1.f;
        glPointSize(ps);
        
        // render points
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        glVertexPointer(3, GL_DOUBLE, 0, &m_points[0]);
        glColorPointer(4, GL_FLOAT, sizeof(cColorf), &m_colors[0]);
        glDrawArrays(GL_POINTS, 0, m_points.size());
        
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        
        // restore OpenGL state
        if (lightingOn) glEnable(GL_LIGHTING);
        if (!blendOn)   glDisable(GL_BLEND);
        glPointSize(pointSize);
    }
}

//! Helper function that computes the direction of minimal co-variance of
//! a set of (relative) position vectors and corresponding weights.
chai3d::cVector3d PointSet::minimizeCovariance(const std::vector<chai3d::cVector3d> &a_positions,
                                               const std::vector<double> &a_weights)
{
    // define standard basis vectors
    const cVector3d zero(0, 0, 0);
    const cVector3d e[3] = {
        cVector3d(1, 0, 0),
        cVector3d(0, 1, 0),
        cVector3d(0, 0, 1)
    };
    
    // first ensure that positions and weights vectors are the same size
    if (a_positions.size() != a_weights.size()) return zero;
    
    // assemble covariance matrix (as per Alexa & Adamson 2004)
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
            for (int i = 0; i < a_positions.size(); ++i)
                W(j, k) += cDot(e[j], a_positions[i] - averagePoint) * cDot(e[k], a_positions[i] - averagePoint) * a_weights[i];
    
    // compute eigenvalues and eigenvectors of covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(W);
    if (solver.info() != Eigen::Success) return zero;
    Eigen::Vector3d lambda = solver.eigenvalues();
    
    // the eigenvalues have already been sorted in increasing order, so the
    // first eigenvector corresponds to the smallest eigenvalue
    cVector3d normal(solver.eigenvectors().col(0));
    return normal;
}




double PointSet::surfaceFunction(cVector3d toolPos)
{
  return (toolPos - averagePoint) * currentNormal;
}

chai3d::cVector3d PointSet::closestPointToPlane(cVector3d toolPos)
{
  cVector3d vec = toolPos - averagePoint;

  double distance = vec * currentNormal;

  return toolPos - currentNormal * distance;
}

void PointSet::setLocalPoints(cVector3d pos)
{
  localPoints = tree->getPointsForArea(pos, radiusOfInfluence);
  
  for (int i : localColorIndices)
  {
    m_colors[i].setRedDark();
  }

  localColorIndices = tree->getCIsForArea(pos, radiusOfInfluence);

  for (int i : localColorIndices)
  {
    m_colors[i].setGreenForest();
  }

}


//===========================================================================
/*!
    This method should contain the core of the implicit surface rendering
    algorithm implementation.  The member variables m_interactionPoint
    and m_interactionInside should both be set by this method.

    \param  a_toolPos  Position of the tool.
    \param  a_toolVel  Velocity of the tool.
    \param  a_IDN  Identification number of the force algorithm.
*/
//===========================================================================
void PointSet::computeLocalInteraction(const cVector3d& a_toolPos,
                                       const cVector3d& a_toolVel,
                                       const unsigned int a_IDN)
{
  // If we are not interacting with the object
  if (!m_interactionInside)
  {
    //find the local points around the tool position
    setLocalPoints(a_toolPos);
    
    // If there are not enough points to calculate a normal assume that we are not in contact with the object
    if (localPoints.size() < 3)
    {
      m_interactionPoint = a_toolPos;
      m_interactionInside = false;
      m_interactionNormal.zero();
      oldNormal.zero();
    }
    // There are enough points to calculate a normal
    else
    {
      // Create the weights vector and a weighted sum of the points
      vector<double> weights;
      averagePoint.zero();
      double weightSum = 0;
      for (cVector3d point : localPoints)
      {
        double w = (a_toolPos - point).length();
        weights.push_back(1 - (w / radiusOfInfluence));

        averagePoint += point * w;
        weightSum += w;
      }
      averagePoint /= weightSum;

      // Calculate a normal
      currentNormal = minimizeCovariance(localPoints, weights);
      // if there is no old normal fix the current normal with the device Position
      if (oldNormal.length() == 0)
        currentNormal = surfaceFunction(a_toolPos) > 0 ? currentNormal : -currentNormal;
      // else fix the new normal witht the old normal
      else
        currentNormal = oldNormal * currentNormal > 0 ? currentNormal : -currentNormal;

      oldNormal = currentNormal;

      surfaceValue = surfaceFunction(a_toolPos);

      // We are not in contact with the object
      if (surfaceValue > 0)
      {
        m_interactionPoint = a_toolPos;
        m_interactionInside = false;
        m_interactionNormal = cVector3d(0, 0, 0);
      }
      // We are in contact with the object
      else
      {
        m_interactionInside = true;
        m_interactionPoint = closestPointToPlane(a_toolPos);
        m_interactionNormal = currentNormal;

        computeLocalInterationInside(a_toolPos, a_toolVel, a_IDN);
      }
    }
  }
  // Else we are in contact with the object
  else
  {
    computeLocalInterationInside(a_toolPos, a_toolVel, a_IDN);
  }

}


void PointSet::computeLocalInterationInside(const chai3d::cVector3d & a_toolPos, const chai3d::cVector3d & a_toolVel, const unsigned int a_IDN)
{

  double mu_s = m_material->getStaticFriction();
  double mu_k = m_material->getDynamicFriction();

  cVector3d delta(0, 0, 0);

  int count = 0;
  int max = 1;
  do {
    // get the local points
    //find the local points around the tool position
    setLocalPoints(m_interactionPoint);

    // If there are not enough points to calculate a normal assume that we are not in contact with the object
    if (localPoints.size() < 3)
    {
      m_interactionPoint = a_toolPos;
      m_interactionInside = false;
      m_interactionNormal.zero();
      oldNormal.zero();
    }
    // There are enough points to calculate a normal
    else
    {
      // Create the weights vector and a weighted sum of the points
      vector<double> weights;
      averagePoint.zero();
      double weightSum = 0;
      for (cVector3d point : localPoints)
      {
        double w = (m_interactionPoint - point).length();
        weights.push_back(1 - (w / radiusOfInfluence));

        averagePoint += point * w;
        weightSum += w;
      }
      averagePoint /= weightSum;

      m_interactionPoint = closestPointToPlane(m_interactionPoint);

      // Calculate a normal
      currentNormal = minimizeCovariance(localPoints, weights);
      // if there is no old normal fix the current normal with the device Position
      currentNormal = oldNormal * currentNormal > 0 ? currentNormal : -currentNormal;

      oldNormal = currentNormal;
      m_interactionNormal = currentNormal;

      surfaceValue = surfaceFunction(a_toolPos);

      // We are not in contact with the object
      if (surfaceValue > 0)
      {
        m_interactionPoint = a_toolPos;
        m_interactionInside = false;
        m_interactionNormal = cVector3d(0, 0, 0);
      }
      // We are in contact with the object
      else
      {
        // find the vector between the tool and the interation point
        chai3d::cVector3d vec = a_toolPos - m_interactionPoint;

        double forceNormal = vec * currentNormal;
        double forcePerp = (vec - forceNormal * currentNormal).length();

        double value = abs(forcePerp / forceNormal);

        chai3d::cVector3d frictionForce = vec - forceNormal * currentNormal;
        frictionForce.normalize();

        frictionForce *= std::min(abs(mu_k * forceNormal), forcePerp);

        // if the cursor is in motion on the object
        if (moving) {
          // 
          if (value > mu_k) {
            // find the closest point on the tangent plane and use to find the closest point on surface


            delta = closestPointToPlane(a_toolPos - frictionForce) - m_interactionPoint;
            m_interactionPoint += delta;
            moving = true;
          }
          //
          else if (value < mu_k * 0.99) {
            moving = false;
          }
        }
        else {
          if (value > mu_s) {
            // find the closest point on the tangent plane and use to find the closest point on surface
            delta = closestPointToPlane(a_toolPos - frictionForce) - m_interactionPoint;
            m_interactionPoint += delta;
            moving = true;

          }
          else if (value < mu_s * 0.99) {
            moving = false;
          }
        }
      }
    }
    ++count;
  } while (delta.length() > 0.0001 && count < max);

}





