#include "simulation.h"
#include <iostream>

using namespace std;
using namespace glm;
using namespace Eigen;

Simulation::Simulation(mutex *renderLock) : renderLock(renderLock) {
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
    VectorXd q, v, qprev;
    cout << "about to buildConfiguration" << endl;
    g_cloth->buildConfiguration(q, v, qprev);
    cout << "about to do numericalIntegration" << endl;
    numericalIntegration(q, v, qprev);
    cout << "about to unpackConfiguration" << endl;
    renderLock->lock();
    g_cloth->unpackConfiguration(q, qprev, v);
    renderLock->unlock();
}

void Simulation::numericalIntegration(VectorXd q, VectorXd v, VectorXd qprev) {
    VectorXd F;
    SparseMatrix<double> H; // the hessian
    F.resize(q.size());
    F.setZero();
    H.resize(q.size(), q.size());
    H.setZero();
    SparseMatrix<double> M;
    g_cloth->computeMassMatrix(M);
    MatrixXd denseM = M.toDense();
    // stretch forces

    // shear and bend forces

    // gravity
    // TODO: allow for fixed particles
    // iterate through the y coords
    for (int i = 1; i < q.size() / 3; i += 3) {
        F[i] += -9.8 * denseM(i/3, i/3); //this should be int div
    }

    // damping
}
