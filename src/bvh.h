#ifndef BVH_H
#define BVH_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include "face.h"
#include "collision.h"
#include "cloth.h"
#include <mutex>

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

        void intersect(Vector3d p, int pIndex, vector<Collision>& collisions, mutex& lock);
};

#endif
