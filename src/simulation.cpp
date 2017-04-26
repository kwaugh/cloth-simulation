#include "simulation.h"
#include <iostream>
#include <math.h>

#define BICBOI BiCGSTAB

using namespace std;
using namespace glm;
using namespace Eigen;

Simulation::Simulation(mutex *renderLock) : renderLock(renderLock) {
    string vCloth = "4";
    g_cloth = make_shared<Cloth>("../src/resources/cloth." + vCloth + ".node",
            "../src/resources/cloth." + vCloth + ".ele", 1, Vector3d(-1, 0, .5));
    g_sphere = make_shared<Sphere>("../src/resources/sphere.node",
            "../src/resources/sphere.ele", 1, Vector3d(0, 0, 1));
}

Simulation::~Simulation() {}

void Simulation::generate_geometry(vector<vec4>& obj_vertices,
        vector<uvec3>& obj_faces, vector<vec4>& obj_normals) {
    obj_vertices.clear();
    obj_faces.clear();
    obj_normals.clear();
    g_cloth->generate_geometry(obj_vertices, obj_faces, obj_normals);
    /* g_sphere->generate_geometry(obj_vertices, obj_faces); */
}

void Simulation::generate_libigl_geometry(MatrixXd& Verts, MatrixXi& Faces) {
    g_cloth->generate_libigl_geometry(Verts, Faces);
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
            break;
        }
        SparseMatrix<double> identity(q.size(), q.size());
        identity.setIdentity();
        SparseMatrix<double> df = (-MatrixXd::Identity(q.size(), q.size())
            + timeStep * timeStep
            * Minv
            * computeDF(guessQ)).sparseView();
        BICBOI<SparseMatrix<double>> solver;
        /* SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver; */
        solver.compute(df);
        guessQ -= solver.solve(f);
    }
    q = guessQ;
    /* q += timeStep * v; */
    F = computeForce(q, qprev);
    v += timeStep * Minv * F;
}

VectorXd Simulation::computeForce(VectorXd q, VectorXd qprev) {
    VectorXd Force_Stretch(q.size());
    Force_Stretch.setZero();
    VectorXd Force_Shear(q.size());
    Force_Shear.setZero();
    VectorXd Force_Bend(q.size());
    Force_Bend.setZero();
    VectorXd massVec = g_cloth->getMassVector();
    auto F = g_cloth->F;
    auto V = g_cloth->V;
    auto Pos = g_cloth->Pos;
    for (int i = 0; i < F.rows(); i++) {
        Vector3d x0 = g_cloth->Pos.row(F(i, 0));
        Vector3d x1 = g_cloth->Pos.row(F(i, 1));
        Vector3d x2 = g_cloth->Pos.row(F(i, 2));
        double deltau1 = V(F(i, 1), 0) - V(F(i, 0), 0);
        double deltau2 = V(F(i, 2), 0) - V(F(i, 0), 0);
        double deltav1 = V(F(i, 1), 1) - V(F(i, 0), 1);
        double deltav2 = V(F(i, 2), 1) - V(F(i, 0), 1);
        double alpha = pow(g_cloth->A[i], .75);
        Vector3d w_u = ((x1 - x0) * deltav2 - (x2 - x0) * deltav1)
            / (deltau1 * deltav2 - deltau2 * deltav1);
        Vector3d w_v = (-(x1 - x0) * deltau2 + (x2 - x0) * deltau1)
            / (deltau1 * deltav2 - deltau2 * deltav1);
        MatrixX3d dwdxx(2, 3);
        dwdxx.setZero();
        dwdxx(0, 0) = (deltav1 - deltav2) / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(0, 1) = (deltav2)           / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(0, 2) = (-deltav1)          / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 0) = (deltau2 - deltau1) / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 1) = (-deltau2)          / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 2) = (deltau1)           / (deltau1 * deltav2 - deltau2 * deltav1);

        /* stretch */ {
            Vector2d C;
            C[0] = alpha * (w_u.norm() - 1);
            C[1] = alpha * (w_v.norm() - 1);
            Vector3d dCudx0 = alpha * dwdxx(0, 0) * w_u.normalized();
            Vector3d dCudx1 = alpha * dwdxx(0, 1) * w_u.normalized();
            Vector3d dCudx2 = alpha * dwdxx(0, 2) * w_u.normalized();

            Vector3d dCvdx0 = alpha * dwdxx(1, 0) * w_v.normalized();
            Vector3d dCvdx1 = alpha * dwdxx(1, 1) * w_v.normalized();
            Vector3d dCvdx2 = alpha * dwdxx(1, 2) * w_v.normalized();

            Force_Stretch.segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCudx0 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCvdx0 * C[1];
            Force_Stretch.segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCudx1 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCvdx1 * C[1];
            Force_Stretch.segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCudx2 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCvdx2 * C[1];
        }
        /* shear */ {
            double C = alpha * w_u.dot(w_v);
            Vector3d dCdx0(
                    alpha * (dwdxx(0, 0) * w_v[0] + dwdxx(1, 0) * w_u[0]),
                    alpha * (dwdxx(0, 0) * w_v[1] + dwdxx(1, 0) * w_u[1]),
                    alpha * (dwdxx(0, 0) * w_v[2] + dwdxx(1, 0) * w_u[2])
            );
            Vector3d dCdx1(
                    alpha * (dwdxx(0, 1) * w_v[0] + dwdxx(1, 1) * w_u[0]),
                    alpha * (dwdxx(0, 1) * w_v[1] + dwdxx(1, 1) * w_u[1]),
                    alpha * (dwdxx(0, 1) * w_v[2] + dwdxx(1, 1) * w_u[2])
            );
            Vector3d dCdx2(
                    alpha * (dwdxx(0, 2) * w_v[0] + dwdxx(1, 2) * w_u[0]),
                    alpha * (dwdxx(0, 2) * w_v[1] + dwdxx(1, 2) * w_u[1]),
                    alpha * (dwdxx(0, 2) * w_v[2] + dwdxx(1, 2) * w_u[2])
            );

            Force_Shear.segment<3>(3 * F(i, 0)) += -g_cloth->kshear * dCdx0 * C;
            Force_Shear.segment<3>(3 * F(i, 1)) += -g_cloth->kshear * dCdx1 * C;
            Force_Shear.segment<3>(3 * F(i, 2)) += -g_cloth->kshear * dCdx2 * C;
        }
        /* /1* bending *1/ { */
        /*     for (uint j = 0; j < g_cloth->adjacentFaces[i].size(); j++) { */
        /*         int i2 = g_cloth->adjacentFaces[i][j]; */
        /*         int unique_i_point = 0; */
        /*         int unique_i2_point = 0; */
        /*         for (int x = 0; x < 3; x++) { */
        /*             if (F(i2, 0) != F(i, x) && */
        /*                 F(i2, 1) != F(i, x) && */
        /*                 F(i2, 2) != F(i, x)) { */
        /*                 unique_i_point = x; */
        /*             } */
        /*             if (F(i, 0) != F(i2, x) && */
        /*                 F(i, 1) != F(i2, x) && */
        /*                 F(i, 2) != F(i2, x)) { */
        /*                 unique_i2_point = x; */
        /*             } */
        /*         } */

        /*         Vector3d x0 = Pos.row(F(i, unique_i_point)); */
        /*         Vector3d x1 = Pos.row(F(i, (unique_i_point+1)%3)); */
        /*         Vector3d x2 = Pos.row(F(i, (unique_i_point+2)%3)); */
        /*         Vector3d x3 = Pos.row(F(i2, unique_i2_point)); */

        /*         int particleIdx[4] = { */
        /*             F(i, unique_i_point), */
        /*             F(i, (unique_i_point+1)%3), */
        /*             F(i, (unique_i_point+2)%3), */
        /*             F(i2, unique_i2_point) */
        /*         }; */

        /*         Vector3d nA = (x2 - x0).cross(x1 - x0); */
        /*         Vector3d nB = (x1 - x3).cross(x2 - x3); */
        /*         Vector3d e = x1 - x2; */

        /*         double sinT = (nA.normalized().cross(nB.normalized()).dot(e.normalized())); */
        /*         double cosT = (nA.normalized().dot(nB.normalized())); */
        /*         double theta = atan2(sinT, cosT); */
        /*         double C = theta; */

        /*         MatrixX3d qA(4, 3); */
        /*         qA.setZero(); */
        /*         qA.row(0) = x2 - x1; */
        /*         qA.row(1) = x0 - x2; */
        /*         qA.row(2) = x1 - x0; */

        /*         MatrixX3d qB(4, 3); */
        /*         qB.setZero(); */
        /*         qB.row(1) = x2 - x3; */
        /*         qB.row(2) = x3 - x1; */
        /*         qB.row(3) = x1 - x2; */

        /*         Vector4d qe(0, 1, -1, 0); */

        /*         for (int m = 0; m < 4; m++) { */
        /*             Vector3d dCdxm; */
        /*             for (int s = 0; s < 3; s++) { */
        /*                 Vector3d dnadxms = S_s(qA.row(m), s); */
        /*                 Vector3d dnbdxms = S_s(qB.row(m), s); */
        /*                 Vector3d dedxms = qe[m] * MatrixXd::Identity(3, 3).row(s); */
        /*                 Vector3d dnhatadxms = dnadxms / nA.norm(); */
        /*                 Vector3d dnhatbdxms = dnbdxms / nB.norm(); */
        /*                 Vector3d dehatdxms = dedxms / e.norm(); */
        /*                 double dcosTdxms = dnhatadxms.dot(nB.normalized()) */
        /*                     + nA.normalized().dot(dnhatbdxms); */
        /*                 double dsinTdxms = ( */
        /*                         dnhatadxms.cross(nB.normalized()) */
        /*                         + nA.normalized().cross(dnhatbdxms) */
        /*                         ).dot(e.normalized()) */
        /*                     + (nA.normalized().cross(nB.normalized()).dot(dehatdxms)); */
        /*                 dCdxm[s] = cosT * dsinTdxms - sinT * dcosTdxms; */
        /*             } */
        /*             Force_Bend.segment<3>(3 * particleIdx[m]) += -g_cloth->kbend */
        /*                 * dCdxm * C; */
        /*         } */
        /*     } */
        /* } */
    }
    /* cout << "Stretch: " << Force_Stretch.segment<3>(0) << endl; */

    /* gravity */
    /* TODO: allow for fixed particles */
    /* iterate through the y coords */
    /* for (int i = 1; i < q.size(); i += 3) { */
    /*     F[i] += -9.8 * denseM(i/3, i/3); //this should be int div */
    /* } */
    VectorXd Force_Gravity(q.size());
    Force_Gravity.setZero();
    for (int i = 1; i < q.size(); i+=3) {
        Force_Gravity[i] += -grav * massVec[i / 3];
    }

    /* cout << "Grav: " << Force_Gravity.segment<3>(0).norm() << endl; */

    /* damping */
    return Force_Gravity + Force_Stretch + Force_Shear + Force_Bend;
}

MatrixXd Simulation::computeDF(VectorXd q) {
    MatrixXd df_stretch(q.size(), q.size());
    df_stretch.setZero();
    MatrixXd df_shear(q.size(), q.size());
    df_shear.setZero();
    MatrixXd df_bend(q.size(), q.size());
    df_bend.setZero();
    auto F = g_cloth->F;
    auto V = g_cloth->V;
    auto Pos = g_cloth->Pos;
    for (int i = 0; i < F.rows(); i++) {
        Vector3d x0 = g_cloth->Pos.row(F(i, 0));
        Vector3d x1 = g_cloth->Pos.row(F(i, 1));
        Vector3d x2 = g_cloth->Pos.row(F(i, 2));
        double deltau1 = V(F(i, 1), 0) - V(F(i, 0), 0);
        double deltau2 = V(F(i, 2), 0) - V(F(i, 0), 0);
        double deltav1 = V(F(i, 1), 1) - V(F(i, 0), 1);
        double deltav2 = V(F(i, 2), 1) - V(F(i, 0), 1);
        double alpha = pow(g_cloth->A[i], .75);
        Vector3d w_u = ((x1 - x0) * deltav2 - (x2 - x0) * deltav1)
            / (deltau1 * deltav2 - deltau2 * deltav1);
        Vector3d w_v = (-(x1 - x0) * deltau2 + (x2 - x0) * deltau1)
            / (deltau1 * deltav2 - deltau2 * deltav1);
        MatrixX3d dwdxx(2, 3);
        dwdxx.setZero();
        dwdxx(0, 0) = (deltav1 - deltav2) / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(0, 1) = (deltav2)           / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(0, 2) = (-deltav1)          / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 0) = (deltau2 - deltau1) / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 1) = (-deltau2)          / (deltau1 * deltav2 - deltau2 * deltav1);
        dwdxx(1, 2) = (deltau1)           / (deltau1 * deltav2 - deltau2 * deltav1);

        /* stretch */ {
            Vector2d C;
            C[0] = alpha * (w_u.norm() - 1);
            C[1] = alpha * (w_v.norm() - 1);

            MatrixX3d dCudx(3, 3);
            MatrixX3d dCvdx(3, 3);
            dCudx.row(0) = alpha * dwdxx(0, 0) * w_u.normalized();
            dCudx.row(1) = alpha * dwdxx(0, 1) * w_u.normalized();
            dCudx.row(2) = alpha * dwdxx(0, 2) * w_u.normalized();

            dCvdx.row(0) = alpha * dwdxx(1, 0) * w_v.normalized();
            dCvdx.row(1) = alpha * dwdxx(1, 1) * w_v.normalized();
            dCvdx.row(2) = alpha * dwdxx(1, 2) * w_v.normalized();

            for (int m = 0; m < 3; m++) {
                for (int n = 0; n < 3; n++) {
                    MatrixX3d d2Cudxmn = alpha / w_u.norm() * dwdxx(0, m)* dwdxx(0, n)
                        * (MatrixXd::Identity(3, 3) - w_u.normalized()*w_u.normalized().transpose());
                    df_stretch.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
                        * (dCudx.row(m).transpose() * dCudx.row(n) + d2Cudxmn * C[0]);

                    MatrixX3d d2Cvdxmn = alpha / w_v.norm() * dwdxx(1, m)* dwdxx(1, n)
                        * (MatrixXd::Identity(3, 3) - w_v.normalized()*w_v.normalized().transpose());
                    df_stretch.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
                        * (dCvdx.row(m).transpose() * dCvdx.row(n) + d2Cvdxmn * C[1]);
                }
            }
        }
        /* shear */ {
            double C = alpha * w_u.dot(w_v);
            MatrixX3d dCdx(3, 3);
            dCdx.row(0) = Vector3d(
                    alpha * (dwdxx(0, 0) * w_v[0] + dwdxx(1, 0) * w_u[0]),
                    alpha * (dwdxx(0, 0) * w_v[1] + dwdxx(1, 0) * w_u[1]),
                    alpha * (dwdxx(0, 0) * w_v[2] + dwdxx(1, 0) * w_u[2])
            );
            dCdx.row(1) = Vector3d(
                    alpha * (dwdxx(0, 1) * w_v[0] + dwdxx(1, 1) * w_u[0]),
                    alpha * (dwdxx(0, 1) * w_v[1] + dwdxx(1, 1) * w_u[1]),
                    alpha * (dwdxx(0, 1) * w_v[2] + dwdxx(1, 1) * w_u[2])
            );
            dCdx.row(2) = Vector3d(
                    alpha * (dwdxx(0, 2) * w_v[0] + dwdxx(1, 2) * w_u[0]),
                    alpha * (dwdxx(0, 2) * w_v[1] + dwdxx(1, 2) * w_u[1]),
                    alpha * (dwdxx(0, 2) * w_v[2] + dwdxx(1, 2) * w_u[2])
            );
            for (int m = 0; m < 3; m++) {
                for (int n = 0; n < 3; n++) {
                    double stretchedD = alpha *
                        (dwdxx(0, m) * dwdxx(1, n) + dwdxx(0, n) * dwdxx(1, m));
                    MatrixX3d d2Cdxmn = MatrixXd::Identity(3, 3) * stretchedD;
                    df_shear.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kshear
                        * (dCdx.row(m).transpose() * dCdx.row(n) + d2Cdxmn * C);
                }
            }
        }
        /* /1* bending *1/ { */
        /*     for (uint j = 0; j < g_cloth->adjacentFaces[i].size(); j++) { */
        /*         int i2 = g_cloth->adjacentFaces[i][j]; */
        /*         int unique_i_point = 0; */
        /*         int unique_i2_point = 0; */
        /*         for (int x = 0; x < 3; x++) { */
        /*             if (F(i2, 0) != F(i, x) && */
        /*                 F(i2, 1) != F(i, x) && */
        /*                 F(i2, 2) != F(i, x)) { */
        /*                 unique_i_point = x; */
        /*             } */
        /*             if (F(i, 0) != F(i2, x) && */
        /*                 F(i, 1) != F(i2, x) && */
        /*                 F(i, 2) != F(i2, x)) { */
        /*                 unique_i2_point = x; */
        /*             } */
        /*         } */

        /*         Vector3d x0 = Pos.row(F(i, unique_i_point)); */
        /*         Vector3d x1 = Pos.row(F(i, (unique_i_point+1)%3)); */
        /*         Vector3d x2 = Pos.row(F(i, (unique_i_point+2)%3)); */
        /*         Vector3d x3 = Pos.row(F(i2, unique_i2_point)); */

        /*         int particleIdx[4] = { */
        /*             F(i, unique_i_point), */
        /*             F(i, (unique_i_point+1)%3), */
        /*             F(i, (unique_i_point+2)%3), */
        /*             F(i2, unique_i2_point) */
        /*         }; */

        /*         Vector3d nA = (x2 - x0).cross(x1 - x0); */
        /*         Vector3d nB = (x1 - x3).cross(x2 - x3); */
        /*         Vector3d e = x1 - x2; */

        /*         double sinT = (nA.normalized().cross(nB.normalized()).dot(e.normalized())); */
        /*         double cosT = (nA.normalized().dot(nB.normalized())); */
        /*         double theta = atan2(sinT, cosT); */
        /*         double C = theta; */

        /*         MatrixX3d qA(4, 3); */
        /*         qA.setZero(); */
        /*         qA.row(0) = x2 - x1; */
        /*         qA.row(1) = x0 - x2; */
        /*         qA.row(2) = x1 - x0; */

        /*         MatrixX3d qB(4, 3); */
        /*         qB.setZero(); */
        /*         qB.row(1) = x2 - x3; */
        /*         qB.row(2) = x3 - x1; */
        /*         qB.row(3) = x1 - x2; */

        /*         Vector4d qe(0, 1, -1, 0); */

        /*         MatrixX4d dqA(4, 4); */
        /*         dqA.setZero(); */
        /*         dqA.row(0) = Vector4d( 0, -1,  1,  0); */
        /*         dqA.row(1) = Vector4d( 1,  0, -1,  0); */
        /*         dqA.row(2) = Vector4d(-1,  1,  0,  0); */
        /*         dqA.row(3) = Vector4d( 0,  0,  0,  0); */

        /*         MatrixX4d dqB(4, 4); */
        /*         dqB.setZero(); */
        /*         dqA.row(0) = Vector4d( 0,  0,  0,  0); */
        /*         dqA.row(1) = Vector4d( 0,  0,  1, -1); */
        /*         dqA.row(2) = Vector4d( 0, -1,  0,  1); */
        /*         dqA.row(3) = Vector4d( 0,  1, -1,  0); */

        /*         for (int m = 0; m < 4; m++) { */
        /*             Vector3d dCdxm; */
        /*             for (int n = 0; n < 4; n++) { */
        /*                 Vector3d dCdxn; */
        /*                 MatrixX3d d2Cdxmn(3, 3); */
        /*                 d2Cdxmn.setZero(); */
        /*                 for (int s = 0; s < 3; s++) { */
        /*                     Vector3d dnadxms = S_s(qA.row(m), s); */
        /*                     Vector3d dnbdxms = S_s(qB.row(m), s); */
        /*                     Vector3d dedxms = qe[m] * MatrixXd::Identity(3, 3).row(s); */
        /*                     Vector3d dnhatadxms = dnadxms / nA.norm(); */
        /*                     Vector3d dnhatbdxms = dnbdxms / nB.norm(); */
        /*                     Vector3d dehatdxms = dedxms / e.norm(); */
        /*                     double dcosTdxms = dnhatadxms.dot(nB.normalized()) */
        /*                         + nA.normalized().dot(dnhatbdxms); */
        /*                     double dsinTdxms = ( */
        /*                             dnhatadxms.cross(nB.normalized()) */
        /*                             + nA.normalized().cross(dnhatbdxms) */
        /*                             ).dot(e.normalized()) */
        /*                         + (nA.normalized().cross(nB.normalized()).dot(dehatdxms)); */
        /*                     dCdxm[s] = cosT * dsinTdxms - sinT * dcosTdxms; */
        /*                     for (int t = 0; t < 3; t++)  { */
        /*                         Vector3d dnadxnt = S_s(qA.row(n), t); */
        /*                         Vector3d dnbdxnt = S_s(qB.row(n), t); */
        /*                         Vector3d dedxnt = qe[n] * MatrixXd::Identity(3, 3).row(t); */
        /*                         Vector3d dnhatadxnt = dnadxnt / nA.norm(); */
        /*                         Vector3d dnhatbdxnt = dnbdxnt / nB.norm(); */
        /*                         Vector3d dehatdxnt = dedxnt / e.norm(); */
        /*                         double dcosTdxnt = dnhatadxnt.dot(nB.normalized()) */
        /*                             + nA.normalized().dot(dnhatbdxnt); */
        /*                         double dsinTdxnt = ( */
        /*                                 dnhatadxnt.cross(nB.normalized()) */
        /*                                 + nA.normalized().cross(dnhatbdxnt) */
        /*                                 ).dot(e.normalized()) */
        /*                             + (nA.normalized().cross(nB.normalized()).dot(dehatdxnt)); */
        /*                         dCdxn[t] = cosT * dsinTdxnt - sinT * dcosTdxnt; */


        /*                         Vector3d dqamdxnt; */
        /*                         dqamdxnt.setZero(); */
        /*                         dqamdxnt[t] = dqA(m, n); */

        /*                         Vector3d dqbmdxnt; */
        /*                         dqbmdxnt.setZero(); */
        /*                         dqbmdxnt[t] = dqB(m, n); */

        /*                         Vector3d d2nadxmsnt = S_s(dqamdxnt, s); */
        /*                         Vector3d d2nbdxmsnt = S_s(dqbmdxnt, s); */

        /*                         Vector3d d2nhatadxmsnt = d2nadxmsnt / nA.norm(); */
        /*                         Vector3d d2nhatbdxmsnt = d2nbdxmsnt / nB.norm(); */
        /*                         /1* Vector3d d2ehatdxmsnt; *1/ */
        /*                         /1* d2ehatdxmsnt.setZero(); *1/ */

        /*                         double d2cosTdxmsnt = d2nhatadxmsnt.dot(nB.normalized()) */
        /*                             + dnhatbdxnt.dot(dnhatadxms) */
        /*                             + dnhatadxnt.dot(dnhatbdxms) */
        /*                             + nA.normalized().dot(d2nhatbdxmsnt); */

        /*                         double d2sinTdxmsnt = ( */
        /*                                 d2nhatadxmsnt.cross(nB.normalized()) */
        /*                                 + dnhatadxms.cross(dnhatbdxnt) */
        /*                                 + dnhatadxnt.cross(dnhatbdxms) */
        /*                                 + nA.normalized().cross(d2nhatbdxmsnt) */
        /*                                 ).dot(e.normalized()) */
        /*                             + ( */
        /*                                     dnhatadxms.cross(nB.normalized()) */
        /*                                     + nA.normalized().cross(dnhatbdxms) */
        /*                               ).dot(dehatdxnt) */
        /*                             + ( */
        /*                                     dnhatadxnt.cross(nB.normalized()) */
        /*                                     + nA.normalized().cross(dnhatbdxnt) */
        /*                               ).dot(dehatdxms); */
        /*                             /1* + (nA.normalized().cross(nB.normalized())) *1/ */
        /*                             /1* * d2ehatdxmsnt; *1/ */

        /*                         d2Cdxmn(s, t) = cosT * d2sinTdxmsnt */
        /*                             - sinT * d2cosTdxmsnt */
        /*                             + (sinT * sinT - cosT * cosT) * */
        /*                                 (dsinTdxms * dcosTdxnt + dcosTdxms * dsinTdxnt) */
        /*                             + 2 * sinT * cosT * */
        /*                                 ( */
        /*                                     dcosTdxms * dcosTdxnt - dsinTdxms * dsinTdxnt */
        /*                                 ); */
                            
        /*                     } */
        /*                 } */
        /*                 df_bend.block<3, 3>(particleIdx[m] * 3, particleIdx[n] * 3) += */
        /*                     -g_cloth->kbend * (dCdxm * dCdxn.transpose() + d2Cdxmn * C); */
        /*             } */
        /*         } */
        /*     } */
        /* } */
    }

    return df_stretch + df_shear + df_bend;
}

const Matrix3d Simulation::S(const Eigen::Vector3d &v) {
    Matrix3d result;
    result << 0, -v[2], v[1],
            v[2], 0, -v[0],
            -v[1], v[0], 0;
    return result;
}
const Vector3d Simulation::S_s(const Eigen::Vector3d &v, int index) {
    return S(v).row(index);
}
