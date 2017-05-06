#ifndef FACE_H
#define FACE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

struct Face {
    Vector3d x1, x2, x3;
    int index, i1, i2, i3;
    Vector3d faceNormal;
    AlignedBox<double, 3> aabb;

    Face() {}
    Face(Vector3d x1, Vector3d x2, Vector3d x3, int index, int i1, int i2, int i3)
        : x1(x1), x2(x2), x3(x3), index(index), i1(i1), i2(i2), i3(i3) {
            faceNormal = (x2-x1).cross(x3-x1).normalized();
        }
    Vector3d getCentroid();
    AlignedBox<double, 3> getAABB(double clothThickness);

};

#endif
