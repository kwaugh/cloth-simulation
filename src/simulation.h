#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include <../src/eigen/Eigen/Core>
#include <../src/eigen/Eigen/Geometry>
#include <glm/glm.hpp>
#include "cloth.h"
#include "sphere.h"
#include <memory>

using namespace Eigen;
using namespace std;

class Simulation {
public:
    Simulation();
    ~Simulation();
    void takeSimulationStep();
    void generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces);

private:
    shared_ptr<Cloth> g_cloth;
    shared_ptr<Sphere> g_sphere;
};

#endif
