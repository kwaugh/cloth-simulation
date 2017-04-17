#ifndef SPHERE_H
#define SPHERE_H

#include <vector>
#include <string>
#include <../src/eigen/Eigen/Core>
#include <../src/eigen/Eigen/Geometry>
#include <glm/glm.hpp>

using namespace Eigen;
using namespace std;

class Sphere {
public:
    Sphere() {}
    Sphere(const string &nodeFilename, const string &eleFilename,
            double scale, Vector3d startPos);
    ~Sphere();
    void generate_geometry(vector<glm::vec4>&,
        vector<glm::uvec3>&) const;
    MatrixX3d V; 
    MatrixX3i F; 
};

#endif
