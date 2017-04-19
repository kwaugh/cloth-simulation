#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include "../src/eigen/Eigen/Core"
#include "../src/eigen/Eigen/Geometry"
#include "../src/eigen/Eigen/Sparse"
#include <glm/glm.hpp>
#include "cloth.h"
#include "sphere.h"
#include <memory>
#include <mutex>

using namespace Eigen;
using namespace std;

class Simulation {
public:
    Simulation(mutex *renderLock);
    ~Simulation();
    void takeSimulationStep();
    void numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces);
    VectorXd computeForce(VectorXd q, VectorXd qprev);
    MatrixXd computeDF(VectorXd q);
    double timeStep = .0003;

private:
    shared_ptr<Cloth> g_cloth;
    shared_ptr<Sphere> g_sphere;
    mutex *renderLock;
    double grav = 9.81;
};

#endif
