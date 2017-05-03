#ifndef COLLISION_H
#define COLLISION_H

#include <Eigen/Core>

using namespace Eigen;

class Collision {
public:
    bool isEdgeEdge;
    Vector3d normal;
    double distance;

    Vector3d x0;
    Vector3d x1;
    Vector3d x2;
    Vector3d x3;

    int p0, p1, p2, p3;

    double a, b, c;
    
    /* Collision(bool isEdgeEdge, Vector3d normal, double distance, Vector3d x0, */
    /*         Vector3d x1, Vector3d x2, Vector3d x3, double a, double b, double c) : isEdgeEdge(isEdgeEdge), normal(normal), distance(distance), x0(x0), x1(x1), x2(x2), x3(x3), a(a), b(b), c(c) { } */
    Collision(bool isEdgeEdge, Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3,
            int p0, int p1, int p2, int p3) :
        isEdgeEdge(isEdgeEdge), x0(x0), x1(x1), x2(x2), x3(x3), p0(p0), p1(p1), p2(p2), p3(p3) {
            a = b = c = -1.0;
            x0 = x1 = x2 = x3 = Vector3d(-1, -1, -1);
        }
};

#endif