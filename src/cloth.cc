#include "cloth.h"
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;
using namespace glm;
using namespace Eigen;

Cloth::Cloth(const string &nodeFilename, const string &eleFilename,
        double scale, Vector3d startPos) {
    ifstream nodeIfs(nodeFilename.c_str());
    if (!nodeIfs) { cout << "dumlaya!" << endl; exit(1); }

    int naeem; // dummy variable
    int numVerts;
    nodeIfs >> numVerts >> naeem >> naeem >> naeem;

    Pos.resize(numVerts, 3);
    V.resize(numVerts, 2);
    double a, b;
    for (int i = 0; i < numVerts; i++) {
        nodeIfs >> naeem >> a >> b >> naeem;
        Pos(i, 0) = (a+1+startPos[0]) * scale / 2.0;
        Pos(i, 1) = startPos[2];
        Pos(i, 2) = (b+1+startPos[1]) * scale / 2.0;
        /* Pos(i, 0) = a; */
        /* Pos(i, 1) = startPos[2]; */
        /* Pos(i, 2) = b; */
        V(i, 0) = a * scale / 2.0; /* TODO: is this valid to mult by scale? */
        V(i, 1) = b * scale / 2.0;
    }

    ifstream eleIfs(eleFilename.c_str());
    int numFaces;
    eleIfs >> numFaces >> naeem >> naeem;
    F.resize(numFaces, 3);
    int x, y, z;
    for (int i = 0; i < numFaces; i++) {
        eleIfs >> naeem >> x >> y >> z;
        F.row(i) = Vector3i(x-1, y-1, z-1);
        Vector3d norm = (Pos.row(x-1)-Pos.row(y-1)).cross(Pos.row(y-1)-Pos.row(z-1));
        if (norm.dot(Vector3d(0, 1, 0)) < 0) {
            F.row(i) = Vector3i(x-1, z-1, y-1);
        }
        /* F.row(i) = Vector3i(x, y, z); */
    }
    Vel.resize(numVerts, 3);
    Vel.setZero();
    oldPos.resize(numVerts, 3);
    oldPos.setZero();

    /* set the masses */
    massVec.resize(numVerts);
    massVec.setZero();
    A.resize(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        Vector3d a, b, c;
        a = Pos.row(F(i, 0));
        b = Pos.row(F(i, 1));
        c = Pos.row(F(i, 2));
        double area = .5 * (b-a).cross(c-a).norm();
        A[i] = area;
        double mass = area * density;
        massVec[F(i, 0)] += mass / 3;
        massVec[F(i, 1)] += mass / 3;
        massVec[F(i, 2)] += mass / 3;
    }

    /* compute mass inverse matrix */
    Minv.resize(Pos.rows() * 3, Pos.rows() * 3);
    vector<Tr> Minvcoeffs;
    for (int i = 0; i < Pos.rows(); i++) {
        Minvcoeffs.push_back(Tr(3*i  , 3*i  , 1.0 / massVec[i]));
        Minvcoeffs.push_back(Tr(3*i+1, 3*i+1, 1.0 / massVec[i]));
        Minvcoeffs.push_back(Tr(3*i+2, 3*i+2, 1.0 / massVec[i]));
    }
    Minv.setFromTriplets(Minvcoeffs.begin(), Minvcoeffs.end());

    /* compute mass matrix */
    M.resize(Pos.rows() * Pos.cols(), Pos.rows() * Pos.cols());
    vector<Tr> Mcoeffs;
    for (int i = 0; i < Pos.rows(); i++) {
        Mcoeffs.push_back(Tr(3*i  , 3*i  , massVec[i]));
        Mcoeffs.push_back(Tr(3*i+1, 3*i+1, massVec[i]));
        Mcoeffs.push_back(Tr(3*i+2, 3*i+2, massVec[i]));
    }
    M.setFromTriplets(Mcoeffs.begin(), Mcoeffs.end());

    /* compute vertex to faces map that's used for rendering */
    vertexToFaces.resize(Pos.rows());
    for (int i = 0; i < F.rows(); i++) {
        vertexToFaces[F(i, 0)].insert(i); 
    }

    /* compute face to adjacent faces map */
    adjacentFaces.resize(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        for (int j = i+1; j < F.rows(); j++) {
            int numShared = 0;
            for (int m = 0; m < 3; m++) {
                for (int n = 0; n < 3; n++) {
                    if (F(i, m) == F(j, n)) {
                        numShared++;
                    }
                }
            }
            if (numShared == 2) {
                adjacentFaces[i].push_back(j);
            }
        }
    }
}

Cloth::~Cloth() { }

void Cloth::generate_geometry(vector<vec4>& obj_vertices,
        vector<uvec3>& obj_faces, vector<vec4>& obj_normals) const {
    for (uint i = 0; i < Pos.rows(); i++) {
        Vector3d row = Pos.row(i);
        obj_vertices.push_back(vec4(row[0], row[1], row[2], 1.0));
    }
    for (uint i = 0; i < F.rows(); i++) {
        Vector3i face = F.row(i);
        obj_faces.push_back(uvec3(face[0], face[1], face[2]));
    }
    /* calculate the vertex normals for rendering purposes */
    for (uint i = 0; i < vertexToFaces.size(); i++) {
        Vector3d vNorm(0, 0, 0);
        for (int face : vertexToFaces[i]) {
            Vector3d x = Pos.row(F(face, 0)); 
            Vector3d y = Pos.row(F(face, 1)); 
            Vector3d z = Pos.row(F(face, 2)); 
            vNorm += (x - y).cross(y - z);
        }
        vNorm = vNorm.normalized();
        obj_normals.push_back(vec4(vNorm[0], vNorm[1], vNorm[2], 0.0));
    }
}

void Cloth::generate_libigl_geometry(MatrixX3d& Verts, MatrixX3i& Faces) {
    Verts = Pos;
    Faces = F;
}

void Cloth::buildConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev) {
    q.resize(Pos.rows() * Pos.cols());
    v.resize(Pos.rows() * Pos.cols());
    qprev.resize(Pos.rows() * Pos.cols());
    for (int i = 0; i < Pos.rows(); i++) {
        q[3*i] = Pos(i, 0);
        q[3*i + 1] = Pos(i, 1);
        q[3*i + 2] = Pos(i, 2);

        v[3*i] = Vel(i, 0);
        v[3*i + 1] = Vel(i, 1);
        v[3*i + 2] = Vel(i, 2);

        qprev[3*i] = oldPos(i, 0);
        qprev[3*i + 1] = oldPos(i, 1);
        qprev[3*i + 2] = oldPos(i, 2);
    }
}
void Cloth::unpackConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev) {
    for (int i = 0; i < Pos.rows(); i++) {
        if (i == 1 || i == 2) continue; // fixed verts
        Pos(i, 0) = q[3*i];
        Pos(i, 1) = q[3*i + 1];
        Pos(i, 2) = q[3*i + 2];

        Vel(i, 0) = v[3*i];
        Vel(i, 1) = v[3*i + 1];
        Vel(i, 2) = v[3*i + 2];

        oldPos(i, 0) = qprev[3*i];
        oldPos(i, 1) = qprev[3*i + 1];
        oldPos(i, 2) = qprev[3*i + 2];
    }
}
