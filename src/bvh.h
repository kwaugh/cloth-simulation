#ifndef BVH_H
#define BVH_H

#include <iostream>
#include "../lib/eigen3/Eigen/Core"
#include "../lib/eigen3/Eigen/Geometry"
#include "../lib/eigen3/Eigen/Sparse"
#include "face.h"
#include "collision.h"
#include "cloth.h"

using namespace std;
using namespace Eigen;

class BVHNode {
    public:
        BVHNode* left;
        BVHNode* right;
        bool isLeaf;
        Face face; // null if !isLeaf
        AlignedBox<double, 3> aabb;
        double clothThickness;

        BVHNode(vector<Face> faces, double clothThickness);
        ~BVHNode();

        void intersect(Vector3d p, int pIndex, vector<Collision>& collisions);
};

#endif
