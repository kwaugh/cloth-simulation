#ifndef CLOTH_H
#define CLOTH_H

#include <vector>
#include <string>
#include "../src/eigen/Eigen/Core"
#include "../src/eigen/Eigen/Geometry"
#include "../src/eigen/Eigen/Sparse"
#include <glm/glm.hpp>
#include <set>

using namespace Eigen;
using namespace std;

typedef Eigen::Triplet<double> Tr;

class Cloth {
public:
    Cloth() {}
    Cloth(const string &nodeFilename, const string &eleFilename,
            double scale, Vector3d startPos);
    ~Cloth();
    void generate_geometry(vector<glm::vec4>&,
        vector<glm::uvec3>&) const;
    MatrixX3d Pos; 
    MatrixX3d oldPos; 
    MatrixX3d Vel; 
    MatrixX3i F; 
    set<int> fixedPoints;
    VectorXd massVec;
    SparseMatrix<double> M;
    SparseMatrix<double> Minv;
     

    void buildConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void unpackConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    SparseMatrix<double> getMassMatrix() { return M; }
    SparseMatrix<double> getInverseMassMatrix() { return Minv; }
    VectorXd getMassVector() { return massVec; }
};
#endif
