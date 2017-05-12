#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glm/glm.hpp>
#include "collision.h"

using namespace Eigen;
using namespace std;

class Object {
public:
    enum ObjectType {
        Sphere,
        Box
    };
    Object() {}
    Object(const string &nodeFilename, const string &eleFilename,
            double scale, Vector3d startPos, int offset, ObjectType type);
    ~Object();
    void intersect(Vector3d p, int pIndex, vector<Collision>* collisions,
            double thickness);
    void generate_libigl_geometry(MatrixX3d&, MatrixX3i&) const;
    double pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3);
    MatrixX3d V; 
    MatrixX3i F; 
    ObjectType type;
    Vector3d startPos;
    double scale;
    int offset;
};

#endif
