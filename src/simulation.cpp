#include "simulation.h"
#include <iostream>
#include <math.h>
#include <utility>
#include <algorithm>
#include <set>
#include <mutex>

#define BICBOI BiCGSTAB

using namespace std;
using namespace glm;
using namespace Eigen;

Simulation::Simulation(mutex& renderLock) : renderLock(renderLock) {
    threadCount = std::max(std::thread::hardware_concurrency(), (unsigned)1);
    threadCount = std::min(threadCount, 10); // don't use hyperthreads
    cout << "threadCount: " << threadCount << endl;
    reset();
}

void Simulation::reset() {
    totalVertices = 0;
    objects.clear();
    g_cloth = make_shared<Cloth>("../src/resources/cloth." + to_string(vCloth) + ".node",
            "../src/resources/cloth." + to_string(vCloth) + ".ele", 1.5 * scale, Vector3d(0, 0, 0));
    totalVertices += g_cloth->Pos.rows();
    objects.push_back(make_shared<Object>("../src/resources/sphere.node",
            "../src/resources/sphere.ele", scale / 4.0, Vector3d(0, -2 * scale, 0), totalVertices,
            Object::ObjectType::Sphere));
    totalVertices += objects[objects.size() - 1]->V.rows();
    objects.push_back(make_shared<Object>("../src/resources/box.node",
            "../src/resources/box.ele", scale / 4.0, Vector3d(0, -scale / 4 - 20, 0), totalVertices,
            Object::ObjectType::Box));
    totalVertices += objects[objects.size() - 1]->V.rows();
    //paused = true;
    stepCount = 0;
    timeStep = 0.003;
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

void Simulation::generate_libigl_geometry(MatrixX3d& Verts, MatrixX3i& Faces,
        VectorXd& C) const {
    g_cloth->generate_libigl_geometry(Verts, Faces, C);
    for (int i = 0; i < objects.size(); i++)
        objects[i]->generate_libigl_geometry(Verts, Faces);
    C.conservativeResize(Faces.rows());
}

void Simulation::takeSimulationStep() {
    if (paused) return;

    stepCount++;
    // cout << "stepCount: " << stepCount << endl;
    VectorXd q_cand, v_cand, qprev, vprev;
    g_cloth->buildConfiguration(q_cand, v_cand, qprev);
    vprev = v_cand;
    /* qprev doesn't get set until this following line */
    numericalIntegration(q_cand, v_cand, qprev);
    g_cloth->Colors.setZero();
    if (COLLISIONS)
        handleCollisions(q_cand, v_cand, qprev, vprev);

    renderLock.lock(); {
        g_cloth->unpackConfiguration(q_cand, v_cand, qprev);
    } renderLock.unlock();
    for (uint i = 0; i < q_cand.size(); i++) {
        if (std::isnan(q_cand[i])) {
            cout << "NaN" << endl;
            exit(1);
        }
    }
}

void Simulation::numericalIntegration(VectorXd &q, VectorXd &v, VectorXd &qprev) {
    VectorXd F;
    F.setZero();
    SparseMatrix<double> H; // the hessian
    H.resize(q.size(), q.size());
    H.setZero();
    SparseMatrix<double> M = g_cloth->getMassMatrix();
    SparseMatrix<double> Minv = g_cloth->getInverseMassMatrix();

    qprev = q;
    VectorXd guessQ = q;
    int i;
    for (i = 0; i < 20; i++) {
        VectorXd f = -guessQ
            + q
            + timeStep * v
            + (
                 timeStep * timeStep
                 * Minv
                 * computeForce(guessQ)
              );
        double residual = f.norm();
        if (residual < 1e-8) {
            break;
        }
        SparseMatrix<double> identity(q.size(), q.size());
        identity.setIdentity();
        SparseMatrix<double> df = (-MatrixXd::Identity(q.size(), q.size())
            + timeStep * timeStep
            * Minv
            * computeDF(guessQ)).sparseView();
        BICBOI<SparseMatrix<double>> solver;
        solver.compute(df);
        VectorXd temp = solver.solve(f);
        guessQ -= temp;
    }
    if (i == 20) {
        timeStep /= 4;
        cout << "stepCount: " << stepCount << "  timeStep: " << timeStep << endl;
        numericalIntegration(q, v, qprev);
        return;
    }
    q = guessQ;
    F = computeForce(q);
    v += timeStep * Minv * F;
}

void Simulation::handleCollisions(VectorXd& q_cand, VectorXd& v_cand,
        VectorXd& qprev, VectorXd& vprev) {
    const VectorXd q_candbak = q_cand;
    const MatrixX3i F = g_cloth->F;
    const MatrixX3d Pos = g_cloth->Pos;
    const VectorXd mass = g_cloth->getMassVector();
    VectorXd v_avg_cand = (q_cand - qprev) / timeStep;

    vector<Collision> collisions;
    vector<Face> faces(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        faces[i] = (Face(
            Pos.row(F(i, 0)),
            Pos.row(F(i, 1)),
            Pos.row(F(i, 2)),
            i,
            F(i, 0),
            F(i, 1),
            F(i, 2)
        ));
    }
    BVHNode root(faces, clothThickness);
    clothClothCollision(collisions, &root);
    /* cloth-object collisions */
    for (int i = 0; i < objects.size(); i++) {
        for (int j = 0; j < objects[i]->V.rows(); j++) {
            root.intersect(objects[i]->V.row(j), -1, &collisions);
        }
    }
    /* cout << "collisions.size(): " << collisions.size() << endl; */

    /* check for sphere/cube collisions */

    VectorXd v_diff_impulse(q_cand.size());
    VectorXd v_diff_spring(q_cand.size());
    VectorXd numImpulses(q_cand.size() / 3);
    VectorXd numSpringForces(q_cand.size() / 3);
    v_diff_impulse.setZero();
    v_diff_spring.setZero();
    numImpulses.setZero();
    numSpringForces.setZero();
    for (int i = 0; i < collisions.size(); i++) {
        Collision c = collisions[i];
        Vector3d vel1(0, 0, 0);
        Vector3d vel2(0, 0, 0);
        if (c.clothCloth || c.p0 == -1) {
            g_cloth->Colors[c.fIndex] += .01;
            vel2 = c.a * vprev.segment<3>(3*c.p1) +
                c.b * vprev.segment<3>(3*c.p2) +
                c.c * vprev.segment<3>(3*c.p3);
        }
        if (c.clothCloth || c.p0 != -1) {
            vel1 = vprev.segment<3>(3*c.p0);
        }
        /* inelastic impulses */ {
            double relvel = vel1.dot(c.normal) - vel2.dot(c.normal);
            if (relvel > 0.0) {
                double Idivm = relvel / (1 + c.a*c.a + c.b*c.b + c.c*c.c);
                if (c.clothCloth) {
                    v_diff_impulse.segment<3>(3*c.p0) += -Idivm * c.normal;
                    v_diff_impulse.segment<3>(3*c.p1) += c.a * Idivm * c.normal;
                    v_diff_impulse.segment<3>(3*c.p2) += c.b * Idivm * c.normal;
                    v_diff_impulse.segment<3>(3*c.p3) += c.c * Idivm * c.normal;
                    numImpulses[c.p0] += 1;
                    numImpulses[c.p1] += 1;
                    numImpulses[c.p2] += 1;
                    numImpulses[c.p3] += 1;
                } else {
                    if (c.p0 != -1) {
                        v_diff_impulse.segment<3>(3*c.p0) += 2 * -Idivm * c.normal;
                        numImpulses[c.p0] += 1;
                    } else {
                        v_diff_impulse.segment<3>(3*c.p1) += 2 * c.a * Idivm * c.normal;
                        v_diff_impulse.segment<3>(3*c.p2) += 2 * c.b * Idivm * c.normal;
                        v_diff_impulse.segment<3>(3*c.p3) += 2 * c.c * Idivm * c.normal;
                        numImpulses[c.p1] += 1;
                        numImpulses[c.p2] += 1;
                        numImpulses[c.p3] += 1;
                    }
                }
            }

        }
        /* repulsion spring force */ {
            double relvel = -(vel1.dot(c.normal) - vel2.dot(c.normal));
            double d = clothThickness - (c.x3 - c.a * c.x0 - c.b * c.x1 - c.c * c.x2).dot(c.normal);
            if (!(relvel >= 0.1 * d / timeStep)) {
                double m1, m2;
                if (c.clothCloth) {
                    m1 = mass[c.p0];
                    m2 = c.a * mass[c.p1] +
                         c.b * mass[c.p2] +
                         c.c * mass[c.p3];
                } else {
                    if (c.p0 != -1) {
                        m1 = m2 = mass[c.p0];
                    } else {
                        m1 = m2 = c.a * mass[c.p1] +
                            c.b * mass[c.p2] +
                            c.c * mass[c.p3];
                    }
                }
                double avgMass = (m1 + m2) / 2;
                double Ir = -1 * std::min(
                        timeStep * g_cloth->kstretch * d,
                        avgMass * (.1 * d / timeStep - relvel)
                        );
                double I = 2 * Ir / (1 + c.a*c.a + c.b*c.b + c.c*c.c);
                if (c.clothCloth) {
                    v_diff_spring.segment<3>(3*c.p0) -= -I / mass[c.p0] * c.normal;
                    v_diff_spring.segment<3>(3*c.p1) -= c.a * I / mass[c.p1] * c.normal;
                    v_diff_spring.segment<3>(3*c.p2) -= c.b * I / mass[c.p2] * c.normal;
                    v_diff_spring.segment<3>(3*c.p3) -= c.c * I / mass[c.p3] * c.normal;
                    numSpringForces[c.p0] += 1;
                    numSpringForces[c.p1] += 1;
                    numSpringForces[c.p2] += 1;
                    numSpringForces[c.p3] += 1;
                } else {
                    /* if (c.p0 != -1) { */
                    /*     v_diff_spring.segment<3>(3*c.p0) -= -I / mass[c.p0] * c.normal; */
                    /*     numSpringForces[c.p0] += 1; */
                    /* } else { */
                    /*     /1* cout << "applying repulsion force" << endl; *1/ */
                    /*     /1* cout << "c.normal: " << c.normal << endl; *1/ */
                    /*     v_diff_spring.segment<3>(3*c.p1) -= c.a * I / mass[c.p1] * c.normal; */
                    /*     v_diff_spring.segment<3>(3*c.p2) -= c.b * I / mass[c.p2] * c.normal; */
                    /*     v_diff_spring.segment<3>(3*c.p3) -= c.c * I / mass[c.p3] * c.normal; */
                    /*     numSpringForces[c.p1] += 1; */
                    /*     numSpringForces[c.p2] += 1; */
                    /*     numSpringForces[c.p3] += 1; */
                    /* } */
                }
            }
        }
    }
    for (int i = 0; i < numImpulses.size(); i++) {
        if (numImpulses[i])
            v_avg_cand.segment<3>(3*i) += v_diff_impulse.segment<3>(3*i) / numImpulses[i];
    }
    for (int i = 0; i < numSpringForces.size(); i++) {
        if (numSpringForces[i])
            v_avg_cand.segment<3>(3*i) += v_diff_spring.segment<3>(3*i) / numSpringForces[i];
    }

    q_cand = qprev + timeStep * v_avg_cand;
    VectorXd oldForces = computeForce(q_candbak);
    VectorXd newForces = computeForce(q_cand);
    VectorXd v_candbak = v_cand;
    if (collisions.size() != 0) {
        /* v_cand = v_avg_cand + timeStep / 2 * g_cloth->getInverseMassMatrix() * newForces; */
        v_cand = v_avg_cand + timeStep * (v_candbak - vprev) / 4;
    }
}

void Simulation::clothClothCollision(vector<Collision>& collisions, BVHNode *root) {
    thread t[threadCount];
    vector<vector<Collision>> threadCollisions(threadCount);
    for (int i = 0; i < threadCount; i++) {
        int end = i == (threadCount - 1) ?
            (int)(g_cloth->Pos.rows()) :
            (int)(g_cloth->Pos.rows() / threadCount) * (i+1);
        t[i] = std::thread(
            &Simulation::clothClothCollisionHelper,
            this,
            (int)((g_cloth->Pos.rows() / threadCount) * i),
            end,
            root,
            &(threadCollisions[i])
        );
    }
    for (int i = 0; i < threadCount; i++) {
        t[i].join();
    }
    for (int i = 0; i < threadCount; i++) {
        collisions.insert(
            collisions.end(),
            threadCollisions[i].begin(),
            threadCollisions[i].end()
        );
    }
}

void Simulation::clothClothCollisionHelper(
        int startRow,
        int endRow,
        BVHNode* root,
        vector<Collision>* collisions) {
    for (int i = startRow; i < endRow; i++) {
        root->intersect(g_cloth->Pos.row(i), i, collisions);
        for (int j = 0; j < objects.size(); j++) {
            objects[j]->intersect(g_cloth->Pos.row(i), i, collisions, clothThickness);
        }
    }
}

VectorXd Simulation::computeForce(VectorXd q) {
    thread t[threadCount];
    vector<VectorXd> stretch(threadCount);
    vector<VectorXd> shear(threadCount);
    vector<VectorXd> bend(threadCount);
    for (int i = 0; i < threadCount; i++) {
        int end = i == (threadCount - 1) ?
            (int)(g_cloth->F.rows()) :
            (int)(g_cloth->F.rows() / threadCount) * (i+1);
        t[i] = std::thread(
            &Simulation::computeForceHelper,
            this,
            (int)q.size(),
            (int)((g_cloth->F.rows() / threadCount) * i),
            end,
            &stretch[i],
            &shear[i],
            &bend[i]
        );
    }
    VectorXd Force_Stretch(q.size());
    VectorXd Force_Shear(q.size());
    VectorXd Force_Bend(q.size());
    VectorXd Force_Gravity(q.size());
    Force_Stretch.setZero();
    Force_Shear.setZero();
    Force_Bend.setZero();
    Force_Gravity.setZero();
    /* compute gravity while we wait for other threads to fiinish */
    if (F_GRAV) {
        VectorXd massVec = g_cloth->getMassVector();
        for (int i = 1; i < q.size(); i+=3) {
            Force_Gravity[i] += -grav * massVec[i / 3];
        }
    }
    for (int i = 0; i < threadCount; i++) {
        t[i].join();
    }
    for (int i = 0; i < threadCount; i++) {
        Force_Stretch += stretch[i];
        Force_Shear += shear[i];
        Force_Bend += bend[i];
    }
    return Force_Stretch + Force_Shear + Force_Bend + Force_Gravity;
}

void Simulation::computeForceHelper(
        int numPoints,
        int startRow,
        int endRow,
        VectorXd* Force_Stretch,
        VectorXd* Force_Shear,
        VectorXd* Force_Bend) const {
    Force_Stretch->resize(numPoints);
    Force_Shear->resize(numPoints);
    Force_Bend->resize(numPoints);

    Force_Stretch->setZero();
    Force_Shear->setZero();
    Force_Bend->setZero();

    auto F = g_cloth->F;
    auto V = g_cloth->V;
    auto Pos = g_cloth->Pos;
    for (int i = startRow; i < endRow; i++) {
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

        if (F_STRETCH) {
            Vector2d C;
            C[0] = alpha * (w_u.norm() - 1);
            C[1] = alpha * (w_v.norm() - 1);
            Vector3d dCudx0 = alpha * dwdxx(0, 0) * w_u.normalized();
            Vector3d dCudx1 = alpha * dwdxx(0, 1) * w_u.normalized();
            Vector3d dCudx2 = alpha * dwdxx(0, 2) * w_u.normalized();

            Vector3d dCvdx0 = alpha * dwdxx(1, 0) * w_v.normalized();
            Vector3d dCvdx1 = alpha * dwdxx(1, 1) * w_v.normalized();
            Vector3d dCvdx2 = alpha * dwdxx(1, 2) * w_v.normalized();

            Force_Stretch->segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCudx0 * C[0];
            Force_Stretch->segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCvdx0 * C[1];
            Force_Stretch->segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCudx1 * C[0];
            Force_Stretch->segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCvdx1 * C[1];
            Force_Stretch->segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCudx2 * C[0];
            Force_Stretch->segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCvdx2 * C[1];
        }
        if (F_SHEAR) {
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

            Force_Shear->segment<3>(3 * F(i, 0)) += -g_cloth->kshear * dCdx0 * C;
            Force_Shear->segment<3>(3 * F(i, 1)) += -g_cloth->kshear * dCdx1 * C;
            Force_Shear->segment<3>(3 * F(i, 2)) += -g_cloth->kshear * dCdx2 * C;
        }
        if (F_BEND) {
            for (uint j = 0; j < g_cloth->adjacentFaces[i].size(); j++) {
                int i2 = g_cloth->adjacentFaces[i][j];
                int unique_i_point = 0;
                int unique_i2_point = 0;
                for (int x = 0; x < 3; x++) {
                    if (F(i2, 0) != F(i, x) &&
                            F(i2, 1) != F(i, x) &&
                            F(i2, 2) != F(i, x)) {
                        unique_i_point = x;
                    }
                    if (F(i, 0) != F(i2, x) &&
                            F(i, 1) != F(i2, x) &&
                            F(i, 2) != F(i2, x)) {
                        unique_i2_point = x;
                    }
                }
                Vector3d x0 = Pos.row(F(i, unique_i_point));
                Vector3d x1 = Pos.row(F(i, (unique_i_point+1)%3));
                Vector3d x2 = Pos.row(F(i, (unique_i_point+2)%3));
                Vector3d x3 = Pos.row(F(i2, unique_i2_point));

                int particleIdx[4] = {
                    F(i, unique_i_point),
                    F(i, (unique_i_point+1)%3),
                    F(i, (unique_i_point+2)%3),
                    F(i2, unique_i2_point)
                };

                Vector3d nA = (x2 - x0).cross(x1 - x0);
                Vector3d nB = (x1 - x3).cross(x2 - x3);
                Vector3d e = x1 - x2;

                double sinT = (nA.normalized().cross(nB.normalized()).dot(e.normalized()));
                double cosT = (nA.normalized().dot(nB.normalized()));
                double theta = atan2(sinT, cosT);
                double C = theta;

                MatrixX3d qA(4, 3);
                qA.setZero();
                qA.row(0) = x2 - x1;
                qA.row(1) = x0 - x2;
                qA.row(2) = x1 - x0;

                MatrixX3d qB(4, 3);
                qB.setZero();
                qB.row(1) = x2 - x3;
                qB.row(2) = x3 - x1;
                qB.row(3) = x1 - x2;

                Vector4d qe(0, 1, -1, 0);

                for (int m = 0; m < 4; m++) {
                    Vector3d dCdxm;
                    for (int s = 0; s < 3; s++) {
                        Vector3d dnadxms = S_s(qA.row(m), s);
                        Vector3d dnbdxms = S_s(qB.row(m), s);
                        Vector3d dedxms = qe[m] * MatrixXd::Identity(3, 3).row(s);
                        Vector3d dnhatadxms = dnadxms / nA.norm();
                        Vector3d dnhatbdxms = dnbdxms / nB.norm();
                        Vector3d dehatdxms = dedxms / e.norm();
                        double dcosTdxms = dnhatadxms.dot(nB.normalized())
                            + nA.normalized().dot(dnhatbdxms);
                        double dsinTdxms = (
                                dnhatadxms.cross(nB.normalized())
                                + nA.normalized().cross(dnhatbdxms)
                                ).dot(e.normalized())
                            + (nA.normalized().cross(nB.normalized()).dot(dehatdxms));
                        dCdxm[s] = cosT * dsinTdxms - sinT * dcosTdxms;
                    }
                    Force_Bend->segment<3>(3 * particleIdx[m]) += -g_cloth->kbend
                        * dCdxm * C;
                }
            }
        }
    }
}

MatrixXd Simulation::computeDF(VectorXd q) {
    thread t[threadCount];
    vector<MatrixXd> stretch(threadCount);
    vector<MatrixXd> shear(threadCount);
    vector<MatrixXd> bend(threadCount);
    for (int i = 0; i < threadCount; i++) {
        int end = i == (threadCount - 1) ?
            (int)(g_cloth->F.rows()) :
            (int)(g_cloth->F.rows() / threadCount) * (i+1);
        t[i] = std::thread(
            &Simulation::computeDFHelper,
            this,
            (int)q.size(),
            (int)((g_cloth->F.rows() / threadCount) * i),
            end,
            &stretch[i],
            &shear[i],
            &bend[i]
        );
    }
    for (int i = 0; i < threadCount; i++) {
        t[i].join();
    }
    MatrixXd df_stretch(q.size(), q.size());
    MatrixXd df_shear(q.size(), q.size());
    MatrixXd df_bend(q.size(), q.size());
    df_stretch.setZero();
    df_shear.setZero();
    df_bend.setZero();
    for (int i = 0; i < threadCount; i++) {
        df_stretch += stretch[i];
        df_shear += shear[i];
        df_bend += bend[i];
    }
    return df_stretch + df_shear + df_bend;
}

void Simulation::computeDFHelper(
        int numPoints,
        int startRow,
        int endRow,
        MatrixXd* df_stretch,
        MatrixXd* df_shear,
        MatrixXd* df_bend
        ) const {
    df_stretch->resize(numPoints, numPoints);
    df_shear->resize(numPoints, numPoints);
    df_bend->resize(numPoints, numPoints);

    df_stretch->setZero();
    df_shear->setZero();
    df_bend->setZero();

    auto F = g_cloth->F;
    auto V = g_cloth->V;
    auto Pos = g_cloth->Pos;
    for (int i = startRow; i < endRow; i++) {
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

        if (F_STRETCH) {
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
                    MatrixX3d d2Cvdxmn = alpha / w_v.norm() * dwdxx(1, m)* dwdxx(1, n)
                        * (MatrixXd::Identity(3, 3) - w_v.normalized()*w_v.normalized().transpose());

                    df_stretch->block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
                        * (dCudx.row(m).transpose() * dCudx.row(n) + d2Cudxmn * C[0]);
                    df_stretch->block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
                        * (dCvdx.row(m).transpose() * dCvdx.row(n) + d2Cvdxmn * C[1]);
                }
            }
        }
        if (F_SHEAR) {
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
                    df_shear->block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kshear
                        * (dCdx.row(m).transpose() * dCdx.row(n) + d2Cdxmn * C);
                }
            }
        }
        if (F_BEND) {
            for (uint j = 0; j < g_cloth->adjacentFaces[i].size(); j++) {
                int i2 = g_cloth->adjacentFaces[i][j];
                int unique_i_point = 0;
                int unique_i2_point = 0;
                for (int x = 0; x < 3; x++) {
                    if (F(i2, 0) != F(i, x) &&
                            F(i2, 1) != F(i, x) &&
                            F(i2, 2) != F(i, x)) {
                        unique_i_point = x;
                    }
                    if (F(i, 0) != F(i2, x) &&
                            F(i, 1) != F(i2, x) &&
                            F(i, 2) != F(i2, x)) {
                        unique_i2_point = x;
                    }
                }

                Vector3d x0 = Pos.row(F(i, unique_i_point));
                Vector3d x1 = Pos.row(F(i, (unique_i_point+1)%3));
                Vector3d x2 = Pos.row(F(i, (unique_i_point+2)%3));
                Vector3d x3 = Pos.row(F(i2, unique_i2_point));

                int particleIdx[4] = {
                    F(i, unique_i_point),
                    F(i, (unique_i_point+1)%3),
                    F(i, (unique_i_point+2)%3),
                    F(i2, unique_i2_point)
                };

                Vector3d nA = (x2 - x0).cross(x1 - x0);
                Vector3d nB = (x1 - x3).cross(x2 - x3);
                Vector3d e = x1 - x2;

                double sinT = (nA.normalized().cross(nB.normalized()).dot(e.normalized()));
                double cosT = (nA.normalized().dot(nB.normalized()));
                double theta = atan2(sinT, cosT);
                double C = theta;

                MatrixX3d qA(4, 3);
                qA.row(0) = x2 - x1;
                qA.row(1) = x0 - x2;
                qA.row(2) = x1 - x0;
                qA.row(3) = Vector3d(0, 0, 0);

                MatrixX3d qB(4, 3);
                qB.row(0) = Vector3d(0, 0, 0);
                qB.row(1) = x2 - x3;
                qB.row(2) = x3 - x1;
                qB.row(3) = x1 - x2;

                Vector4d qe(0, 1, -1, 0);

                MatrixX4d dqA(4, 4);
                dqA.row(0) = Vector4d( 0, -1,  1,  0);
                dqA.row(1) = Vector4d( 1,  0, -1,  0);
                dqA.row(2) = Vector4d(-1,  1,  0,  0);
                dqA.row(3) = Vector4d( 0,  0,  0,  0);

                MatrixX4d dqB(4, 4);
                dqB.row(0) = Vector4d( 0,  0,  0,  0);
                dqB.row(1) = Vector4d( 0,  0,  1, -1);
                dqB.row(2) = Vector4d( 0, -1,  0,  1);
                dqB.row(3) = Vector4d( 0,  1, -1,  0);

                for (int m = 0; m < 4; m++) {
                    Vector3d dCdxm;
                    for (int n = m; n < 4; n++) {
                        Vector3d dCdxn;
                        MatrixX3d d2Cdxmn(3, 3);
                        for (int s = 0; s < 3; s++) {
                            Vector3d dnadxms = S_s(qA.row(m), s);
                            Vector3d dnbdxms = S_s(qB.row(m), s);
                            Vector3d dedxms = qe[m] * MatrixXd::Identity(3, 3).row(s);
                            Vector3d dnhatadxms = dnadxms / nA.norm();
                            Vector3d dnhatbdxms = dnbdxms / nB.norm();
                            Vector3d dehatdxms = dedxms / e.norm();
                            if (nA.norm() < 0.0000000001) cout << "nA is zero" << endl;
                            if (nB.norm() < 0.0000000001) cout << "nB is zero" << endl;
                            if (e.norm() < 0.0000000001) cout << "e is zero" << endl;
                            double dcosTdxms = dnhatadxms.dot(nB.normalized())
                                + nA.normalized().dot(dnhatbdxms);
                            double dsinTdxms = (
                                    dnhatadxms.cross(nB.normalized())
                                    + nA.normalized().cross(dnhatbdxms)
                                    ).dot(e.normalized())
                                + (nA.normalized().cross(nB.normalized()).dot(dehatdxms));
                            dCdxm[s] = cosT * dsinTdxms - sinT * dcosTdxms;
                            for (int t = s; t < 3; t++)  {
                                Vector3d dnadxnt = S_s(qA.row(n), t);
                                Vector3d dnbdxnt = S_s(qB.row(n), t);
                                Vector3d dedxnt = qe[n] * MatrixXd::Identity(3, 3).row(t);
                                Vector3d dnhatadxnt = dnadxnt / nA.norm();
                                Vector3d dnhatbdxnt = dnbdxnt / nB.norm();
                                Vector3d dehatdxnt = dedxnt / e.norm();
                                double dcosTdxnt = dnhatadxnt.dot(nB.normalized())
                                    + nA.normalized().dot(dnhatbdxnt);
                                double dsinTdxnt = (
                                        dnhatadxnt.cross(nB.normalized())
                                        + nA.normalized().cross(dnhatbdxnt)
                                        ).dot(e.normalized())
                                    + (nA.normalized().cross(nB.normalized()).dot(dehatdxnt));
                                dCdxn[t] = cosT * dsinTdxnt - sinT * dcosTdxnt;


                                Vector3d dqamdxnt;
                                dqamdxnt.setZero();
                                dqamdxnt[t] = dqA(m, n);

                                Vector3d dqbmdxnt;
                                dqbmdxnt.setZero();
                                dqbmdxnt[t] = dqB(m, n);

                                Vector3d d2nadxmsnt = S_s(dqamdxnt, s);
                                Vector3d d2nbdxmsnt = S_s(dqbmdxnt, s);

                                Vector3d d2nhatadxmsnt = d2nadxmsnt / nA.norm();
                                Vector3d d2nhatbdxmsnt = d2nbdxmsnt / nB.norm();

                                double d2cosTdxmsnt = d2nhatadxmsnt.dot(nB.normalized())
                                    + dnhatbdxnt.dot(dnhatadxms)
                                    + dnhatadxnt.dot(dnhatbdxms)
                                    + nA.normalized().dot(d2nhatbdxmsnt);

                                double d2sinTdxmsnt = (
                                        d2nhatadxmsnt.cross(nB.normalized())
                                        + dnhatadxms.cross(dnhatbdxnt)
                                        + dnhatadxnt.cross(dnhatbdxms)
                                        + nA.normalized().cross(d2nhatbdxmsnt)
                                        ).dot(e.normalized())
                                    + (
                                            dnhatadxms.cross(nB.normalized())
                                            + nA.normalized().cross(dnhatbdxms)
                                      ).dot(dehatdxnt)
                                    + (
                                            dnhatadxnt.cross(nB.normalized())
                                            + nA.normalized().cross(dnhatbdxnt)
                                      ).dot(dehatdxms);

                                d2Cdxmn(s, t) = cosT * d2sinTdxmsnt
                                    - sinT * d2cosTdxmsnt
                                    + (sinT * sinT - cosT * cosT) *
                                    (dsinTdxms * dcosTdxnt + dcosTdxms * dsinTdxnt)
                                    + 2 * sinT * cosT *
                                    (
                                     dcosTdxms * dcosTdxnt - dsinTdxms * dsinTdxnt
                                    );
                                d2Cdxmn(t, s) = d2Cdxmn(s, t);
                            }
                        }
                        df_bend->block<3, 3>(particleIdx[m] * 3, particleIdx[n] * 3) +=
                            -g_cloth->kbend * (dCdxm * dCdxn.transpose() + d2Cdxmn * C);
                    }
                }
            }
        }
    }
}

const Matrix3d Simulation::S(const Eigen::Vector3d &v) {
    Matrix3d result;
    result << 0,   -v[2], v[1],
    v[2], 0,   -v[0],
    -v[1], v[0], 0;
    return result;
}
const Vector3d Simulation::S_s(const Eigen::Vector3d &v, int index) {
    return S(v).row(index);
}

double Simulation::pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector3d unitNorm = (x2 - x1).cross(x3 - x1).normalized();
    return abs(unitNorm.dot(x0 - x1));
}

bool Simulation::pointTriIntersection(Collision& coll) {
    Vector3d unitNorm = (coll.x2 - coll.x1).cross(coll.x3 - coll.x1).normalized();
    double dist = unitNorm.dot(coll.x0 - coll.x1);
    Vector3d projectedPoint = coll.x0 - (dist * unitNorm);
    Vector3d bary = Cloth::getBary(projectedPoint, coll.x1, coll.x2, coll.x3);

    coll.normal = unitNorm;
    coll.a = bary[0];
    coll.b = bary[1];
    coll.c = bary[2];

    return coll.a > 0 && coll.b > 0 && coll.c > 0;
}
