#ifndef MENGER_H
#define MENGER_H

#include <vector>
#include <string>
#include <../src/eigen/Eigen/Core>
#include <glm/glm.hpp>

using namespace Eigen;
using namespace std;

class Cloth {
public:
    Cloth() {}
    Cloth(const string &nodeFilename, const string &eleFilename,
            double scale, Vector3d startPos);
    ~Cloth();
    void generate_geometry(vector<glm::vec4>&,
        vector<glm::uvec3>&) const;
    MatrixX3d V; 
    MatrixX3i F; 
};

#endif
