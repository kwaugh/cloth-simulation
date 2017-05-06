#ifndef COLLISION_H
#define COLLISION_H

#include "../lib/eigen3/Eigen/Core"

using namespace Eigen;

class Collision {
public:
    bool isEdgeEdge;
    Vector3d x0, x1, x2, x3;
    int p0, p1, p2, p3;
    int fIndex;

    double a, b, c;
    Vector3d normal;
    double distance;

    Collision(bool isEdgeEdge, Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3,
            int p0, int p1, int p2, int p3) :
        isEdgeEdge(isEdgeEdge), x0(x0), x1(x1), x2(x2), x3(x3), p0(p0), p1(p1), p2(p2), p3(p3) {
            a = b = c = -1.0;
            normal = Vector3d(0, 0, 0);
            distance = -1.0;
        }
};

#endif
