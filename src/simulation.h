#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include "../lib/eigen3/Eigen/Core"
#include "../lib/eigen3/Eigen/Geometry"
#include "../lib/eigen3/Eigen/Sparse"
#include <glm/glm.hpp>
#include "cloth.h"
#include "sphere.h"
#include <memory>
#include <mutex>

using namespace Eigen;
using namespace std;

class Simulation {
public:
    Simulation();
    ~Simulation();
    void takeSimulationStep();
    void numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces, vector<glm::vec4>& obj_normals);
    void generate_libigl_geometry(MatrixX3d&, MatrixX3i&) const;
    VectorXd computeForce(VectorXd q, VectorXd qprev);
    MatrixXd computeDF(VectorXd q);
    static const Eigen::Matrix3d S(const Eigen::Vector3d &v);
    static const Eigen::Vector3d S_s(const Eigen::Vector3d &v, int index);
    void reset();
    /* x0 is point, x1-x3 is plane */
    double pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3);

    double timeStep = .0003;
    double grav = 9.81;
    bool F_GRAV = true;
    bool F_STRETCH = true;
    bool F_SHEAR = true;
    bool F_BEND = false;
    bool paused = false;
    int vCloth = 5;

    int runCount = 0;

private:
    shared_ptr<Cloth> g_cloth;
    shared_ptr<Sphere> g_sphere;
};

#endif
