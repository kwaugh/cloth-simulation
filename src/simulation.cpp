#include "simulation.h"
#include <iostream>
#include <math.h>
#include <utility>
#include <algorithm>
#include "vec.h"
#include "rootparitycollisiontest.h"
#include <set>
#include "bvh.h"

#define BICBOI BiCGSTAB

using namespace std;
using namespace glm;
using namespace Eigen;

/* void generateStringIds(string a, string b, string c, string d, set<string>& uniqueIds) { */
/*     uniqueIds.insert(a + "." + b + "." + c + "." + d); */
/*     uniqueIds.insert(b + "." + a + "." + c + "." + d); */
/*     uniqueIds.insert(a + "." + b + "." + d + "." + c); */
/*     uniqueIds.insert(b + "." + a + "." + d + "." + c); */

/*     uniqueIds.insert(c + "." + d + "." + a + "." + b); */
/*     uniqueIds.insert(d + "." + c + "." + a + "." + b); */
/*     uniqueIds.insert(c + "." + d + "." + b + "." + a); */
/*     uniqueIds.insert(d + "." + c + "." + b + "." + a); */
/* } */

Simulation::Simulation() {
    reset();
}

void Simulation::reset() {
    g_cloth = make_shared<Cloth>("../src/resources/cloth." + to_string(vCloth) + ".node",
            "../src/resources/cloth." + to_string(vCloth) + ".ele", scale, Vector3d(-1, 0, .5));
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

void Simulation::generate_libigl_geometry(MatrixX3d& Verts, MatrixX3i& Faces,
        VectorXd& C) const {
    g_cloth->generate_libigl_geometry(Verts, Faces, C);
}

void Simulation::takeSimulationStep() {
    stepCount++;
    VectorXd q_cand, v_cand, qprev, vprev;
    g_cloth->buildConfiguration(q_cand, v_cand, qprev);
    vprev = v_cand;
    /* qprev doesn't get set until this following line */
    numericalIntegration(q_cand, v_cand, qprev);
    VectorXd q_candbak = q_cand;
    /* check for collisions */
    MatrixX3i F = g_cloth->F;
    MatrixX3d Pos = g_cloth->Pos;
    VectorXd mass = g_cloth->getMassVector();
    g_cloth->Colors.setZero();
    /* map<int, int> pointFaceCollisions; /1* point index, face index *1/ */
    /* map<pair<int, int>, pair<int, int>> edgeEdgeCollisions; */
    if (COLLISIONS) {
        handleCollisions(q_cand, v_cand, qprev, vprev);
    }

    g_cloth->unpackConfiguration(q_cand, v_cand, qprev);
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
            /* qprev = q; */
            break;
        }
        SparseMatrix<double> identity(q.size(), q.size());
        identity.setIdentity();
        SparseMatrix<double> df = (-MatrixXd::Identity(q.size(), q.size())
            + timeStep * timeStep
            * Minv
            * computeDF(guessQ)).sparseView();
        /* cout << "df: " << df << endl; */
        BICBOI<SparseMatrix<double>> solver;
        /* SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver; */
        /* cout << "q.size(): " << q.size() << endl; */
        solver.compute(df);
        VectorXd temp = solver.solve(f);
        /* cout << temp << endl; */
        guessQ -= solver.solve(f);
    }
    /* cout << "Newton's method ran in " << i << " iterations." << endl; */
    q = guessQ;
    /* q += timeStep * v; */
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
    vector<Face> faces;
    for (int i = 0; i < F.rows(); i++) {
        faces.push_back(Face(
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
    for (int i = 0; i < Pos.rows(); i++) {
        root.intersect(Pos.row(i), i, collisions);
    }
    /* cout << "collisions.size(): " << collisions.size() << endl; */
    /* Using Eigen's BVH */
    /* vector<Face> faces; */
    /* vector<AlignedBox<double, 3>> aabbs; */
    /* for (int i = 0; i < F.rows(); i++) { */
    /*     faces.push_back(Face( */
    /*         Pos.row(F(i, 0)), */
    /*         Pos.row(F(i, 1)), */
    /*         Pos.row(F(i, 2)), */
    /*         i, */
    /*         F(i, 0), */
    /*         F(i, 1), */
    /*         F(i, 2) */
    /*     )); */
    /*     aabbs.push_back(faces[i].getAABB()); */
    /* } */
    /* KdBVH<double, 3, Face> bvh(faces.begin(), faces.end(), aabbs.begin(), aabbs.end()); */
    /* for (int i = 0; i < Pos.rows(); i++) { */
    /*     Intersector inter(Pos.row(i), i, clothThickness); */
    /*     BVIntersect(bvh, inter); */
    /*     collisions.insert(collisions.end(), inter.collisions.begin(), inter.collisions.end()); */
    /* } */
    /* cout << "collisions.size(): " << collisions.size() << endl; */

    /* set<string> uniqueCollisions; */

    /* check for vertex-face collisions */
    /* for (int i = 0; i < F.rows(); i++) { */
    /*     for (int j = 0; j < q_cand.size() / 3; j++) { */
    /*         if (j == F(i, 0) || j == F(i, 1) || j == F(i, 2)) */
    /*             continue; */
    /*         Collision coll( */
    /*             false, */
    /*             Pos.row(j), */
    /*             Pos.row(F(i, 0)), */
    /*             Pos.row(F(i, 1)), */
    /*             Pos.row(F(i, 2)), */
    /*             j, F(i, 0), F(i, 1), F(i, 2) */
    /*         ); */
    /*         coll.distance = */
    /*             pointPlaneDist(Pos.row(j), Pos.row(F(i, 0)), Pos.row(F(i, 1)), Pos.row(F(i, 2))); */
    /*         bool intersected = pointTriIntersection(coll); */
    /*         if (coll.distance < clothThickness && intersected) { */
    /*             collisions.push_back(coll); */
    /*             /1* g_cloth->Colors[i] += .01; *1/ */
    /*         } */
    /*     } */
    /* } */
    /* check for edge-edge collisions */
    /* for (uint i = 0; i < F.rows(); i++) { */
    /*     for (uint j = i+1; j < F.rows(); j++) { */
    /*         for (int k = 0; k < 3; k++) { */
    /*             for (int l = 0; l < 3; l++) { */
    /*                 string id = to_string(F(i, k)) + "." + to_string(F(i, (k+1)%3)) + "." + */
    /*                     to_string(F(j, l)) + "." + to_string(F(j, (l+1)%3)); */
    /*                 if (F(i, k) == F(j, l) || F(i, k) == F(j, (l+1)%3) || */
    /*                         F(i, (k+1)%3) == F(j, l) || F(i, (k+1)%3) == F(j, (l+1)%3) || */
    /*                         uniqueCollisions.count(id) */
    /*                     ) */
    /*                     continue; */
    /*                 Collision coll( */
    /*                     true, */
    /*                     Pos.row(F(i, k)), */
    /*                     Pos.row(F(i, (k+1)%3)), */
    /*                     Pos.row(F(j, l)), */
    /*                     Pos.row(F(j, (l+1)%3)), */
    /*                     F(i, k), */
    /*                     F(i, (k+1)%3), */
    /*                     F(j, l), */
    /*                     F(j, (l+1)%3) */
    /*                 ); */
    /*                 generateStringIds(to_string(F(i, k)), to_string(F(i, (k+1)%3)), */
    /*                     to_string(F(j, l)), to_string(F(j, (l+1)%3)), uniqueCollisions); */
    /*                 edgeEdgeIntersection(coll); */
    /*                 if (coll.distance < clothThickness) { */
    /*                     collisions.push_back(coll); */
    /*                     g_cloth->Colors[i] += .01; */
    /*                     g_cloth->Colors[j] += .01; */
    /*                     /1* cout << "adding collision: " << F(i, k) << " " << F(i, (k+1)%3) << " " << F(j, l) << " " << F(j, (l+1)%3) << endl; *1/ */
    /*                     /1* cout << "collisions.size(): " << collisions.size() << endl; *1/ */
    /*                     /1* cout << "i: " << i << "  j: " << j << endl; *1/ */
    /*                 } */
    /*             } */
    /*         } */
    /*     } */
    /* } */

    VectorXd v_diff_impulse(q_cand.size());
    VectorXd v_diff_spring(q_cand.size());
    VectorXd numImpulses(q_cand.size() / 3);
    VectorXd numSpringForces(q_cand.size() / 3);
    v_diff_impulse.setZero();
    v_diff_spring.setZero();
    numImpulses.setZero();
    numSpringForces.setZero();
    for (Collision c : collisions) {
        g_cloth ->Colors[c.fIndex] += .01;
        if (c.isEdgeEdge) {
            /* Vector3d vel1 = c.a * vprev.segment<3>(3*c.p1) + (1 - c.a) * vprev.segment<3>(3*c.p0); */
            /* Vector3d vel2 = c.b * vprev.segment<3>(3*c.p3) + (1 - c.b) * vprev.segment<3>(3*c.p2); */
            /* /1* double mass1 = c.a * mass[c.p1] + (1 - c.a) * mass[c.p0]; *1/ */
            /* /1* double mass2 = c.b * mass[c.p3] + (1 - c.b) * mass[c.p2]; *1/ */

            /* // Relative velocity magnitude in the normal direction */
            /* double relvel = vel1.dot(c.normal) - vel2.dot(c.normal); */

            /* // They're headed towards each other */
            /* cout << "relvel: " << relvel << "  stepCount: " << stepCount << endl; */
            /* if (relvel > 0.0) { */
            /*     double Idivm = relvel / (c.a*c.a + (1-c.a)*(1-c.a) + c.b*c.b + (1-c.b)*(1-c.b)); */
            /*     /1* cout << "c.p0: " << c.p0 << "  c.p1: " << c.p1 << "  c.p2: " << c.p2 << "  c.p3: " << c.p3 << endl; *1/ */
            /*     /1* cout << "Idivm: " << Idivm << endl; *1/ */
            /*     /1* cout << "c.normal: " << c.normal << endl; *1/ */
            /*     cout << "c.a: " << c.a << "  c.b: " << c.b << endl; */
            /*     v_avg_cand_diff_unscaled.segment<3>(3*c.p0) += (1 - c.a) * Idivm * c.normal; */
            /*     v_avg_cand_diff_unscaled.segment<3>(3*c.p1) += c.a * Idivm * c.normal; */
            /*     v_avg_cand_diff_unscaled.segment<3>(3*c.p2) += -(1 - c.b) * Idivm * c.normal; */
            /*     v_avg_cand_diff_unscaled.segment<3>(3*c.p3) += -c.b * Idivm * c.normal; */
            /*     numImpulses[c.p0] += 1; */
            /*     numImpulses[c.p1] += 1; */
            /*     numImpulses[c.p2] += 1; */
            /*     numImpulses[c.p3] += 1; */
            /* } */
        } else {
            Vector3d vel1 = vprev.segment<3>(3*c.p0);
            Vector3d vel2 = c.a * vprev.segment<3>(3*c.p1) +
                            c.b * vprev.segment<3>(3*c.p2) +
                            c.c * vprev.segment<3>(3*c.p3);
            double relvel = vel1.dot(c.normal) - vel2.dot(c.normal);

            if (relvel > 0.0) {
                double Idivm = relvel / (1 + c.a*c.a + c.b*c.b + c.c*c.c);
                v_diff_impulse.segment<3>(3*c.p0) += -Idivm * c.normal;
                v_diff_impulse.segment<3>(3*c.p1) += c.a * Idivm * c.normal;
                v_diff_impulse.segment<3>(3*c.p2) += c.b * Idivm * c.normal;
                v_diff_impulse.segment<3>(3*c.p3) += c.c * Idivm * c.normal;
                numImpulses[c.p0] += 1;
                numImpulses[c.p1] += 1;
                numImpulses[c.p2] += 1;
                numImpulses[c.p3] += 1;
                /* cout << "ey beta. stepCount: " << stepCount << "  numCollisions: " << collisions.size() << endl; */
            }
        }

        // Do the repulsion spring force
        if (c.isEdgeEdge) {

        } else {
            Vector3d vel1 = vprev.segment<3>(3*c.p0);
            Vector3d vel2 = c.a * vprev.segment<3>(3*c.p1) +
                            c.b * vprev.segment<3>(3*c.p2) +
                            c.c * vprev.segment<3>(3*c.p3);
            double relvel = vel1.dot(c.normal) - vel2.dot(c.normal);
            double d = clothThickness - (c.x3 - c.a * c.x0 - c.b * c.x1 - c.c * c.x2).dot(c.normal);
            /* cout << "relvel: " << relvel << "  other: " << .1 * d / timeStep << endl; */
            if (-relvel >= 0.1 * d / timeStep) {
                cout << "wazzzuhhh dude?" << endl;
            } else {
                double m1 = mass[c.p0];
                double m2 = c.a * mass[c.p1] +
                    c.b * mass[c.p2] +
                    c.c * mass[c.p3];
                double avgMass = (m1 + m2) / 2;
                double Ir = -1 * std::min(
                    timeStep * g_cloth->kstretch * d,
                    avgMass * (.1 * d / timeStep + relvel)
                );
                double I = 2 * Ir / (1 + c.a*c.a + c.b*c.b + c.c*c.c);
                cout << "repulsion spring force: " << I << endl;
                v_diff_spring.segment<3>(3*c.p0) += -I / mass[c.p0] * c.normal;
                v_diff_spring.segment<3>(3*c.p1) += c.a * I / mass[c.p1] * c.normal;
                v_diff_spring.segment<3>(3*c.p2) += c.b * I / mass[c.p2] * c.normal;
                v_diff_spring.segment<3>(3*c.p3) += c.c * I / mass[c.p3] * c.normal;

                numSpringForces[c.p0] += 1;
                numSpringForces[c.p1] += 1;
                numSpringForces[c.p2] += 1;
                numSpringForces[c.p3] += 1;
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
    /* cout << "q_cand - q_candbak: " << (q_cand - q_candbak).norm() << endl; */
    VectorXd oldForces = computeForce(q_candbak);
    VectorXd newForces = computeForce(q_cand);
    /* cout << "newForces - oldForces: " << (newForces - oldForces).norm() << endl; */
    VectorXd v_candbak = v_cand;
    if (collisions.size() != 0) {
        /* v_cand = v_avg_cand + timeStep / 2 * g_cloth->getInverseMassMatrix() * newForces; */
        v_cand = v_avg_cand + timeStep * (v_candbak - vprev) / 4;
    }
    /* v_cand = v_avg_cand + timeStep / 2 * (v_candbak - vprev) / timeStep; */
    /* cout << "v_cand - v_candbak: " << (v_cand - v_candbak).norm() << endl; */
    /* cout << endl; */
}

VectorXd Simulation::computeForce(VectorXd q) {
    VectorXd Force_Stretch(q.size());
    VectorXd Force_Shear(q.size());
    VectorXd Force_Bend(q.size());
    VectorXd Force_Gravity(q.size());

    Force_Stretch.setZero();
    Force_Shear.setZero();
    Force_Bend.setZero();
    Force_Gravity.setZero();

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

            Force_Stretch.segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCudx0 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 0)) += -g_cloth->kstretch * dCvdx0 * C[1];
            Force_Stretch.segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCudx1 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 1)) += -g_cloth->kstretch * dCvdx1 * C[1];
            Force_Stretch.segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCudx2 * C[0];
            Force_Stretch.segment<3>(3 * F(i, 2)) += -g_cloth->kstretch * dCvdx2 * C[1];
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

            Force_Shear.segment<3>(3 * F(i, 0)) += -g_cloth->kshear * dCdx0 * C;
            Force_Shear.segment<3>(3 * F(i, 1)) += -g_cloth->kshear * dCdx1 * C;
            Force_Shear.segment<3>(3 * F(i, 2)) += -g_cloth->kshear * dCdx2 * C;
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
                /* cout << "F.row(i): " << F.row(i) << endl; */
                /* cout << "F.row(i2): " << F.row(i2) << endl; */
                /* if (i == i2) { */
                /*     cout << "i == i2" << endl; */
                /* } */

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
                    Force_Bend.segment<3>(3 * particleIdx[m]) += -g_cloth->kbend
                        * dCdxm * C;
                }
            }
        }
    }
    /* cout << "Stretch: " << Force_Stretch.segment<3>(0) << endl; */

    /* gravity */
    /* TODO: allow for fixed particles */
    /* iterate through the y coords */
    /* for (int i = 1; i < q.size(); i += 3) { */
    /*     F[i] += -9.8 * denseM(i/3, i/3); //this should be int div */
    /* } */
    if (F_GRAV) {
        for (int i = 1; i < q.size(); i+=3) {
            Force_Gravity[i] += -grav * massVec[i / 3];
        }
    }

    /* cout << "Grav: " << Force_Gravity.segment<3>(0).norm() << endl; */

    /* damping */
    return Force_Gravity + Force_Stretch + Force_Shear + Force_Bend;
}

MatrixXd Simulation::computeDF(VectorXd q) {
    MatrixXd df_stretch(q.size(), q.size());
    MatrixXd df_shear(q.size(), q.size());
    MatrixXd df_bend(q.size(), q.size());

    df_stretch.setZero();
    df_shear.setZero();
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
                    df_stretch.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
                        * (dCudx.row(m).transpose() * dCudx.row(n) + d2Cudxmn * C[0]);

                    MatrixX3d d2Cvdxmn = alpha / w_v.norm() * dwdxx(1, m)* dwdxx(1, n)
                        * (MatrixXd::Identity(3, 3) - w_v.normalized()*w_v.normalized().transpose());
                    df_stretch.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kstretch
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
                    df_shear.block<3, 3>(F(i, m) * 3, F(i, n) * 3) += -g_cloth->kshear
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
                /* cout << "nA.norm(): " << nA.norm() << endl; */
                /* cout << "nB.norm(): " << nB.norm() << endl; */
                /* cout << "e.norm(): " << e.norm() << endl; */
                /* if (nB.norm() > 5) { */
                /*     cout << "x0: " << x0 << endl; */
                /*     cout << "x1: " << x1 << endl; */
                /*     cout << "x2: " << x2 << endl; */
                /*     cout << "x3: " << x3 << endl; */
                /* } */

                double sinT = (nA.normalized().cross(nB.normalized()).dot(e.normalized()));
                double cosT = (nA.normalized().dot(nB.normalized()));
                double theta = atan2(sinT, cosT);
                /* cout << "theta: " << theta << endl; */
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
                                /* Vector3d d2ehatdxmsnt; */
                                /* d2ehatdxmsnt.setZero(); */

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
                                    /* + (nA.normalized().cross(nB.normalized())) */
                                    /* * d2ehatdxmsnt; */

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
                        df_bend.block<3, 3>(particleIdx[m] * 3, particleIdx[n] * 3) +=
                            -g_cloth->kbend * (dCdxm * dCdxn.transpose() + d2Cdxmn * C);
                    }
                }
            }
        }
    }

    return df_stretch + df_shear + df_bend;
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

Vec3d convert(Vector3d v) {
   return Vec3d(v[0], v[1], v[2]);
}

void Simulation::edgeEdgeIntersection(Collision& coll) {

    const double EPSILON = 0.0000001;
    Vector3d v1 = coll.x1 - coll.x0;
    Vector3d v2 = coll.x3 - coll.x2;
    //Check if these are parallel
    if (v1.cross(v2).norm() < EPSILON) {
        /* cout << "TODO: handle case when edges are almost parallel" << endl; */
        coll.distance = 999999;
    } else {
        MatrixX2d distMat(2, 2);
        distMat.row(0) = Vector2d(v1.dot(v1), -v1.dot(v2));
        distMat.row(1) = Vector2d(-v1.dot(v2), v2.dot(v2));
        Vector2d ab = distMat.inverse() * Vector2d(v1.dot(coll.x2 - coll.x0), -v2.dot(coll.x2-coll.x0));

        double aClamped = clamp(ab[0], 0.0, 1.0);
        double bClamped = clamp(ab[1], 0.0, 1.0);

        Vector3d finalA, finalB;
        /* TODO: is this necessary? */
        /* if (abs(aClamped - ab[0]) < EPSILON && abs(bClamped - ab[1]) < EPSILON) { */
        /*     finalA = coll.x0 + ab[0] * v1; */
        /*     finalB = coll.x2 + ab[1] * v2; */
        /* } else if (abs(aClamped - ab[0]) > abs(bClamped - ab[1])) { */
        if (abs(aClamped - ab[0]) > abs(bClamped - ab[1])) {
            finalA = coll.x0 + aClamped * v1;
            coll.a = aClamped;
            Vector3d unclampedfinalB = coll.x2 + (finalA-coll.x2).dot(v2) / v2.dot(v2) * v2;

            if ((coll.x3 - unclampedfinalB).dot(coll.x2 - unclampedfinalB) > 0) {
                if ((coll.x3 - unclampedfinalB).norm() > (coll.x2 - unclampedfinalB).norm()) {
                    finalB = coll.x2;
                    coll.b = 0.0;
                } else {
                    finalB = coll.x3;
                    coll.b = 1.0;
                }
                /* finalB = ((coll.x3 - unclampedfinalB).norm() > (coll.x2 - unclampedfinalB).norm()) ? */
                /*     coll.x2 : coll.x3; */
            } else {
                finalB = unclampedfinalB;
                coll.b = (finalB - coll.x2).norm() / (coll.x3 - coll.x2).norm();
            }
        } else {
            finalB = coll.x2 + bClamped * v2;
            coll.b = bClamped;
            Vector3d unclampedfinalA = coll.x0 + (finalB-coll.x0).dot(v1) / v1.dot(v1) * v1;

            if ((coll.x1 - unclampedfinalA).dot(coll.x0 - unclampedfinalA) > 0) {
                if ((coll.x1 - unclampedfinalA).norm() > (coll.x0 - unclampedfinalA).norm()) {
                    finalA = coll.x0;
                    coll.a = 0.0;
                } else {
                    finalA = coll.x1;
                    coll.a = 1.0;
                }
            } else {
                finalA = unclampedfinalA;
                coll.a = (finalA - coll.x0).norm() / (coll.x1 - coll.x0).norm();
            }
        }
        coll.normal = (finalB - finalA).normalized();
        coll.distance = (finalA - finalB).norm();
    }
}

/* bool Intersector::intersectVolume(AlignedBox<double, 3> aabb) { */
/*     return aabb.contains(p); */
/* } */

/* bool Intersector::intersectObject(Face f) { */
/*     Collision c(false, p, f.x1, f.x2, f.x3, f.index, f.i1, f.i2, f.i3); */
/*     c.distance = Simulation::pointPlaneDist(p, f.x1, f.x2, f.x3); */
/*     c.fIndex = f.index; */
/*     bool intersected = pointTriIntersection(c); */
/*     if (c.distance < clothThickness && intersected) { */
/*         collisions.push_back(c); */
/*         return true; */
/*     } */
/*     return false; */
/* } */

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
