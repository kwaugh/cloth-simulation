#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <glm/glm.hpp>
#include "cloth.h"
#include "object.h"
#include "collision.h"
#include <memory>
#include <mutex>
#include <thread>

using namespace Eigen;
using namespace std;

class Simulation {
public:
    Simulation(mutex& renderLock);
    ~Simulation();
    void takeSimulationStep();
    void numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces, vector<glm::vec4>& obj_normals);
    void generate_libigl_geometry(MatrixX3d&, MatrixX3i&, VectorXd&) const;
    VectorXd computeForce(VectorXd q);
    void computeForceHelper(
        int numPoints,
        int startRow,
        int endRow,
        VectorXd& Force_Stretch,
        VectorXd& Force_Shear,
        VectorXd& Force_Bend,
        VectorXd& Force_Gravity
    );
    MatrixXd computeDF(VectorXd q);
    void handleCollisions(VectorXd& q_cand, VectorXd& v_cand, VectorXd& qprev,
            VectorXd& vprev);
    static const Eigen::Matrix3d S(const Eigen::Vector3d &v);
    static const Eigen::Vector3d S_s(const Eigen::Vector3d &v, int index);
    void reset();
    /* x0 is point, x1-x3 is plane */
    static double pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3);
    static bool pointTriIntersection(Collision& coll);
    /* double edgeEdgeDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3); */
    void edgeEdgeIntersection(Collision& coll);

    double timeStep = .003;
    double grav     = 9.81;

    bool F_GRAV     = true;
    bool F_STRETCH  = true;
    bool F_SHEAR    = true;
    bool F_BEND     = true;
    bool COLLISIONS = true;
    bool VIS_COLLISIONS = false;

    bool paused     = false;
    int vCloth      = 1;
    double scale    = 2000;
    double clothThickness = 0.01 * scale;

    int runCount = 0;
    long long stepCount = 0;
    int totalVertices = 0;

    shared_ptr<Cloth> g_cloth;
    vector<shared_ptr<Object>> objects;
    mutex lock;
    mutex& renderLock;
    int threadCount;
};

#endif
