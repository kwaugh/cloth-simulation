#include "object.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace glm;
using namespace Eigen;

double Object::pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector3d unitNorm = (x2 - x1).cross(x3 - x1).normalized();
    return abs(unitNorm.dot(x0 - x1));
}

Object::Object(const string &nodeFilename, const string &eleFilename,
        double scale, Vector3d startPos, int offset, ObjectType type)
        : startPos(startPos), scale(scale), offset(offset), type(type) {
    ifstream nodeIfs(nodeFilename.c_str());
    if (!nodeIfs) { cout << "Couldn't find object node/ele files!" << endl; exit(1); }

    int naeem; // dummy variable
    int numVerts;
    nodeIfs >> numVerts >> naeem >> naeem >> naeem;
    /* cout << "numVerts: " << numVerts << endl; */

    V.resize(numVerts, 3);
    double a, b, c;
    for (int i = 0; i < numVerts; i++) {
        nodeIfs >> a >> b >> c;
        V.row(i) = Vector3d(a, b, c) * scale + startPos;
    }

    ifstream eleIfs(eleFilename.c_str());
    int numFaces;
    eleIfs >> numFaces >> naeem >> naeem;
    /* cout << "numFaces: " << numFaces << endl; */
    F.resize(numFaces, 3);
    int x, y, z;
    cout << "offset: " << offset << endl;
    for (int i = 0; i < numFaces; i++) {
        eleIfs >> x >> y >> z;
        F.row(i) = Vector3i(x + offset, y + offset, z + offset);
        /* Vector3d norm = (V.row(x-1) - V.row(y-1)).cross(V.row(y-1) - V.row(z-1)); */
        /* if (norm.dot(Vector3d(0, 1, 0)) < 0) { */
        /*     F.row(i) = Vector3i(x-1, z-1, y-1); */
        /* } */
        /* cout << norm.dot(Vector3d(0,1,0)) << endl << endl; */
        /* F.row(i) = Vector3i(x, y, z); */
    }
}

Object::~Object() { }

void Object::generate_libigl_geometry(MatrixX3d& Verts, MatrixX3i& Faces) const {
    int oldVRows = Verts.rows();
    int oldFRows = Faces.rows();
    Verts.conservativeResize(oldVRows + V.rows(), 3);
    Faces.conservativeResize(oldFRows + F.rows(), 3);
    Verts.block(oldVRows, 0, V.rows(), 3) = V;
    Faces.block(oldFRows, 0, F.rows(), 3) = F;
}

void Object::intersect(Vector3d p, int pIndex, vector<Collision>& collisions,
        mutex& lock, double thickness) {
    switch (type) {
        case Sphere: {
            /* cout << "startPos: " << startPos << endl; */
            /* cout << "(p - startPos).norm(): " << (p - startPos).norm() << "\nscale + thickness: " << scale + thickness << endl; */
            if ((p - startPos).norm() > scale + thickness) return;
            Collision c;
            c.distance = (p - startPos).norm() - scale;
            c.normal = (startPos - p).normalized();
            c.p0 = pIndex;
            c.x0 = p;
            c.a = 1;
            c.b = c.c = 0;
            lock.lock(); {
                collisions.push_back(c);
            } lock.unlock();
            break;
        }
        case Box: {
            for (int i = 0; i < 3; i++) {
                if (abs(p[i] - startPos[i]) > scale + thickness) return;    
            }
            double minDist = 3 * thickness;
            int index = 0;
            for (int i = 0; i < F.rows(); i++) {
                double dist = pointPlaneDist(
                    p,
                    V.row(F(i, 0) - offset),
                    V.row(F(i, 1) - offset),
                    V.row(F(i, 2) - offset)
                );  
                if (dist < minDist) {
                    minDist = dist;
                    index = i;
                }
            }
            Collision c;
            c.distance = minDist;
            c.normal = (V.row(F(index, 1) - offset) - V.row(F(index, 0) - offset))
                 .cross(V.row(F(index, 2) - offset) - V.row(F(index, 1) - offset))
                 .normalized();
            c.p0 = pIndex;
            c.x0 = p;
            c.a = 1;
            c.b = c.c = 0;
            lock.lock(); {
                collisions.push_back(c);
            } lock.unlock();
            break;
        }
    } 
}
