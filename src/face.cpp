#include "face.h"

AlignedBox<double, 3> Face::getAABB(double clothThickness) {
    Vector3d minVal, maxVal;
    for (int i = 0; i < 3; i++) {
        double delta = abs((clothThickness * faceNormal)[i]);
        minVal[i] = std::min(
            std::min(
                x1[i] - delta,
                x2[i] - delta
            ),
            x3[i] - delta
        );
        maxVal[i] = std::max(
            std::max(
                x1[i] + delta,
                x2[i] + delta
            ),
            x3[i] + delta
        );
    }
    AlignedBox<double, 3> aabb = AlignedBox<double, 3>(minVal, maxVal);
    /* assert(aabb.contains(x1)); */
    /* assert(aabb.contains(x2)); */
    /* assert(aabb.contains(x3)); */
    /* assert(aabb.contains(x1 + clothThickness * faceNormal)); */
    /* assert(aabb.contains(x1 - clothThickness * faceNormal)); */
    /* assert(aabb.contains(x2 + clothThickness * faceNormal)); */
    /* assert(aabb.contains(x2 - clothThickness * faceNormal)); */
    /* assert(aabb.contains(x3 + clothThickness * faceNormal)); */
    /* assert(aabb.contains(x3 - clothThickness * faceNormal)); */
    return aabb;
}

Vector3d Face::getCentroid() {
    return (x1 + x2 + x3) / 3.0;
}
