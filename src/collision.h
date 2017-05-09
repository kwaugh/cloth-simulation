#ifndef COLLISION_H
#define COLLISION_H

#include <Eigen/Core>
#include <iostream>

using namespace Eigen;

class Collision {
public:
    bool clothCloth = true; /* the face will only be set if this is true */
    Vector3d x0, x1, x2, x3;
    int p0, p1, p2, p3;
    int fIndex;

    double a, b, c;
    Vector3d normal;
    double distance;

    Collision() : clothCloth(false) {}
    Collision(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3,
            int p0, int p1, int p2, int p3) :
        x0(x0), x1(x1), x2(x2), x3(x3), p0(p0), p1(p1), p2(p2), p3(p3) {
            a = b = c = -1.0;
            normal = Vector3d(0, 0, 0);
            distance = -1.0;
        }
    friend std::ostream& operator<<(std::ostream &strm, const Collision &c) {
        return strm << "clothCloth: " << c.clothCloth <<
            "\n\tx0: " << c.x0 <<
            "\n\tx1: " << c.x1 <<
            "\n\tx2: " << c.x2 <<
            "\n\tx3: " << c.x3 <<
            "\n\tx3: " << c.x3 <<
            "\n\tp0: " << c.p0 <<
            "\n\tp1: " << c.p1 <<
            "\n\tp2: " << c.p2 <<
            "\n\tp3: " << c.p3 <<
            "\n\tfIndex: " << c.fIndex <<
            "\n\ta: " << c.a <<
            "\n\tb: " << c.b <<
            "\n\tc: " << c.c <<
            "\n\tnormal: " << c.normal <<
            "\n\tdistance: " << c.distance << std::endl;
    }
};

#endif
