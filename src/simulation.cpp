#include "simulation.h"

using namespace std;
using namespace glm;
using namespace Eigen;

Simulation::Simulation() {
    /* g_cloth = make_shared<Cloth>("../src/resources/cloth.node", */
    /*         "../src/resources/cloth.ele", 1, Vector3d(0, 0, 1)); */
    g_cloth = make_shared<Cloth>("../src/resources/cloth.1.node",
            "../src/resources/cloth.1.ele", 1, Vector3d(0, 0, 1));
    g_sphere = make_shared<Sphere>("../src/resources/sphere.node",
            "../src/resources/sphere.ele", 1, Vector3d(0, 0, 1));
}

Simulation::~Simulation() {}

void Simulation::generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces) {
    g_cloth->generate_geometry(obj_vertices, obj_faces);
    g_sphere->generate_geometry(obj_vertices, obj_faces);
}

void Simulation::takeSimulationStep() {

}
