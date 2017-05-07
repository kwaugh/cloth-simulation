#include "bvh.h"

bool pointTriIntersection(Collision& coll) {
    Vector3d unitNorm = (coll.x2 - coll.x1).cross(coll.x3 - coll.x1).normalized();
    double dist = unitNorm.dot(coll.x0 - coll.x1);
    Vector3d projectedPoint = coll.x0 - (dist * unitNorm);
    Vector3d bary = Cloth::getBary(projectedPoint, coll.x1, coll.x2, coll.x3);

    coll.normal = unitNorm;
    coll.a = bary[0];
    coll.b = bary[1];
    coll.c = bary[2];

    return coll.a > 0 && coll.b > 0 && coll.c > 0;
}

double pointPlaneDist(Vector3d x0, Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector3d unitNorm = (x2 - x1).cross(x3 - x1).normalized();
    return abs(unitNorm.dot(x0 - x1));
}

class FaceComparator {
    public:
	FaceComparator(int axis) : axis(axis) {}
	int axis;

	bool operator()(Face& left, Face& right) {
            return left.getCentroid()[axis] < right.getCentroid()[axis];
	}
};

BVHNode::BVHNode(vector<Face> faces, double clothThickness) : clothThickness(clothThickness) {
    if (faces.size() == 0) {
        cout << "faces.size() == 0" << endl;
    }
    if (faces.size() == 1) {
	isLeaf = true;
	face = faces[0];
	aabb = face.getAABB(clothThickness);
    } else {
        isLeaf = false;
	Vector3d minCorner, maxCorner;
        AlignedBox<double, 3> faceAABB = faces[0].getAABB(clothThickness);
        minCorner = faceAABB.min();
        maxCorner = faceAABB.max();
	for (uint i = 1; i < faces.size(); i++) {
	    faceAABB = faces[i].getAABB(clothThickness);
	    for (int j = 0; j < 3; j++) {
		minCorner[j] = std::min(minCorner[j], faceAABB.min()[j]);
		maxCorner[j] = std::max(maxCorner[j], faceAABB.max()[j]);
	    }
	} 
	aabb = AlignedBox<double, 3>(minCorner, maxCorner);

	/* find longest axis */
	int longestAxis = 0;
	double longestAxisVal = maxCorner[0] - minCorner[0];
	for (int i = 1; i < 3; i++) {
	    double axisVal = maxCorner[i] - minCorner[i];
	    if (axisVal > longestAxisVal) {
		longestAxis = i;
		longestAxisVal = axisVal;
	    }
	}
        std::sort(faces.begin(), faces.end(), FaceComparator(longestAxis));
        vector<Face> lList(faces.size() / 2);
        vector<Face> rList(faces.size() - faces.size() / 2);
        for (uint i = 0; i < lList.size(); i++) {
            lList[i] = faces[i];
        }
        for (uint i = 0; i < rList.size(); i++) {
            rList[i] = faces[lList.size() + i];
        }

        left = new BVHNode(lList, clothThickness);
        right = new BVHNode(rList, clothThickness);
        assert(aabb.contains(left->aabb));
        assert(aabb.contains(right->aabb));
    }
}

BVHNode::~BVHNode() {
    if (!isLeaf) {
        delete left;
        delete right;
    }
}

void BVHNode::intersect(Vector3d p, int pIndex, vector<Collision>& collisions, mutex& lock) {
    if (!aabb.contains(p)) return;
    if (!isLeaf) {
	left->intersect(p, pIndex, collisions, lock);
	right->intersect(p, pIndex, collisions, lock);
    } else {
        if (pIndex == face.i1 || pIndex == face.i2 || pIndex == face.i3) return;
	Collision coll(
            p,
            face.x1,
            face.x2,
            face.x3,
            pIndex,
            face.i1,
            face.i2,
            face.i3
        );
	coll.distance = pointPlaneDist(p, face.x1, face.x2, face.x3);
	if (pointTriIntersection(coll) && coll.distance < clothThickness) {
            coll.fIndex = face.index;
            lock.lock(); {
                collisions.push_back(coll);
            } lock.unlock();
	}
    }
}
