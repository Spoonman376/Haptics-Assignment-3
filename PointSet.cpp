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
}

//! Contains code to load a point set with color from a PLY file
bool PointSet::loadFromFile(std::string a_filename)
{
    using namespace std;
    bool result = false;
    
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
            float x, y, z, r, g, b;
            getline(in, line);
            istringstream iss(line);
            iss >> x >> y >> z >> r >> g >> b;
            points->newVertex(cVector3d(x, y, z));
            points->m_vertices->setColor(i, r/255.f, g/255.f, b/255.f);
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
    
    //tree = new SphereTree(m_points);

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
                W(j, k) += cDot(e[j], a_positions[i]) * cDot(e[k], a_positions[i]) * a_weights[i];
    
    // compute eigenvalues and eigenvectors of covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(W);
    if (solver.info() != Eigen::Success) return zero;
    Eigen::Vector3d lambda = solver.eigenvalues();
    
    // the eigenvalues have already been sorted in increasing order, so the
    // first eigenvector corresponds to the smallest eigenvalue
    cVector3d normal(solver.eigenvectors().col(0));
    return normal;
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
    /////////////////////////////////////////////////////////////////////////
    // CPSC.86  UNSTRUCTURED POINT SET RENDERING ALGORITHM
    /////////////////////////////////////////////////////////////////////////
    
    // the positions of the points in the point set can be accessed in the
    // member variable m_points, of type std::vector<cVector3d>
    
    // [start by inserting your code from you previous assignment here]
    
    // m_interactionProjectedPoint is the "proxy" point on the surface
    // that the rendering algorithm tracks.  It should be equal to the
    // tool position when the tool is not in contact with the object.
   // m_interactionPoint = a_toolPos;

    // m_interactionInside should be set to true when the tool is in contact
    // with the object.
    m_interactionInside = false;
    
    double radiusOfInfluence = 0.8;
    std::vector<cVector3d> localPoints;

    for (int i = 0; i < m_points.size(); ++i) {
      cVector3d p = m_points[i];

     // m_colors[i].setRed();

      if (cDistance(p, m_interactionPoint) < radiusOfInfluence) {
        localPoints.push_back(p - m_interactionPoint);
        //m_colors[i].setGreenLimeGreen();
      }

      if (cDistance(p, a_toolPos) < 0.15) {
        m_interactionInside = true;
        m_interactionNormal = cNormalize(m_interactionPoint - p);
        //[i].setBlueCadet();
      }

    }

    if (!m_interactionInside) {
      m_interactionPoint = a_toolPos;
      m_interactionNormal.zero();
    }
    

    std::vector<double> weights(localPoints.size(), 1.0);

    cVector3d normal = minimizeCovariance(localPoints, weights);
    m_interactionNormal = normal;

    // set the surface normal at the interaction point to help visualize the
    // tangent plane
    //if (m_interactionInside && !cEqualPoints(m_interactionPoint, a_toolPos, 0.0))
    //   m_interactionNormal = cNormalize(m_interactionPoint - a_toolPos);
    //else m_interactionNormal.zero();
}
