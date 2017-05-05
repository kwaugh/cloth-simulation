#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include "../lib/eigen3/Eigen/Core"
#include "../lib/eigen3/Eigen/Geometry"
#include "../lib/eigen3/Eigen/Sparse"
#include "../lib/eigen3/unsupported/Eigen/BVH"
/* #include <Eigen/Core> */
/* #include <Eigen/Geometry> */
/* #include <Eigen/Sparse> */
/* #include <unsupported/Eigen/BVH> */
#include <glm/glm.hpp>
#include "cloth.h"
#include "sphere.h"
#include "collision.h"
#include <memory>
#include <mutex>

using namespace Eigen;
using namespace std;

class Simulation;

struct Face {
    Vector3d x1, x2, x3;
    int index, i1, i2, i3;

    Face() {}
    Face(Vector3d x1, Vector3d x2, Vector3d x3, int index, int i1, int i2, int i3)
        : x1(x1), x2(x2), x3(x3), index(index), i1(i1), i2(i2), i3(i3) {}

    AlignedBox<double, 3> getAABB();
};

struct Intersector {
    Vector3d p;
    int index;
    vector<Collision> collisions;
    double clothThickness;

    Intersector(Vector3d p, int index, double clothThickness)
        : p(p), index(index), clothThickness(clothThickness) { }

    bool intersectVolume(AlignedBox<double, 3> aabb);
    bool intersectObject(Face f);
    bool pointTriIntersection(Collision& coll);
};

class Simulation {
public:
    Simulation();
    ~Simulation();
    void takeSimulationStep();
    void numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces, vector<glm::vec4>& obj_normals);
    void generate_libigl_geometry(MatrixX3d&, MatrixX3i&, VectorXd&) const;
    VectorXd computeForce(VectorXd q);
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

    bool paused     = false;
    int vCloth      = 5;
    double scale       = 30;
    double clothThickness = 0.01 * scale;

    int runCount = 0;
    long long stepCount = 0;


private:
    shared_ptr<Cloth> g_cloth;
    shared_ptr<Sphere> g_sphere;
};

#endif
