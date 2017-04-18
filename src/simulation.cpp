#include "simulation.h"
#include <iostream>

#define BICBOI BiCGSTAB 

using namespace std;
using namespace glm;
using namespace Eigen;

Simulation::Simulation(mutex *renderLock) : renderLock(renderLock) {
    g_cloth = make_shared<Cloth>("../src/resources/cloth.node",
            "../src/resources/cloth.ele", 1, Vector3d(0, 0, 1));
    /* g_cloth = make_shared<Cloth>("../src/resources/cloth.2.node", */
    /*         "../src/resources/cloth.2.ele", 1, Vector3d(0, 0, 1)); */
    /* g_cloth = make_shared<Cloth>("../src/resources/cloth.1.node", */
    /*         "../src/resources/cloth.1.ele", 1, Vector3d(0, 0, 1)); */
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
    g_cloth->buildConfiguration(q, v, qprev);
    numericalIntegration(q, v, qprev);
    renderLock->lock();
    g_cloth->unpackConfiguration(q, v, qprev);
    renderLock->unlock();
}

void Simulation::numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev) {
    VectorXd F;
    F.setZero();
    SparseMatrix<double> H; // the hessian
    H.resize(q.size(), q.size());
    H.setZero();
    SparseMatrix<double> M = g_cloth->getMassMatrix();
    SparseMatrix<double> Minv = g_cloth->getInverseMassMatrix();

    VectorXd guessQ = q;
    for (int i = 0; i < 20; i++) {
        VectorXd f = -guessQ
            + q
            + timeStep * v
            + (
                 timeStep * timeStep
                 * Minv
                 * computeForce(guessQ, qprev)
              );
        double residual = f.norm();
        if (residual < 1e-8) {
            /* qprev = q; */
            q = guessQ;
            break;
        }
        SparseMatrix<double> identity(q.size(), q.size());
        identity.setIdentity();
        SparseMatrix<double> df = -identity
            + timeStep * timeStep
            * Minv
            * computeDF(guessQ);
        BICBOI<SparseMatrix<double>> solver;
        /* SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver; */
        solver.compute(df);
        guessQ -= solver.solve(f);
    }
    F = computeForce(q, qprev);
    v += timeStep * Minv * F;
}

VectorXd Simulation::computeForce(VectorXd q, VectorXd qprev) {
    VectorXd F(q.size());
    F.setZero();
    VectorXd massVec = g_cloth->getMassVector();
    /* stretch forces */

    /* shear and bend forces */

    /* gravity */
    /* TODO: allow for fixed particles */
    /* iterate through the y coords */
    /* for (int i = 1; i < q.size(); i += 3) { */
    /*     F[i] += -9.8 * denseM(i/3, i/3); //this should be int div */
    /* } */
    for (int i = 1; i < q.size(); i+=3) {
        F[i] += -9.8 * massVec[i / 3]; 
    }

    /* damping */
    return F;
}

MatrixXd Simulation::computeDF(VectorXd q) {
    MatrixXd df(q.size(), q.size());
    df.setZero();
    /* gravity = 0 */ 

    return df;
}
