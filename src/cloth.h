#ifndef CLOTH_H
#define CLOTH_H

#include <vector>
#include <string>
#include "../lib/eigen3/Eigen/Core"
#include "../lib/eigen3/Eigen/Geometry"
#include "../lib/eigen3/Eigen/Sparse"
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
        vector<glm::uvec3>&, vector<glm::vec4>&) const;
    void generate_libigl_geometry(MatrixX3d&, MatrixX3i&, VectorXd&) const;
    MatrixX3d Pos; 
    MatrixX2d V; 
    MatrixX3d oldPos; 
    MatrixX3d Vel; 
    MatrixX3i F; 
    VectorXd A;
    VectorXd Colors;
    set<int> fixedPoints;
    VectorXd massVec;
    SparseMatrix<double> M;
    SparseMatrix<double> Minv;

    vector<set<int>> vertexToFaces;
    vector<vector<int>> adjacentFaces;

    static Vector3d getBary(Vector3d point, Vector3d a, Vector3d b, Vector3d c);
     

    void buildConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    void unpackConfiguration(VectorXd &q, VectorXd &v, VectorXd &qprev);
    SparseMatrix<double> getMassMatrix() { return M; }
    SparseMatrix<double> getInverseMassMatrix() { return Minv; }
    VectorXd getMassVector() { return massVec; }

    const double density = 0.1;
    const double kstretch = 5.0e3;
    const double kshear = 0.5e3;
    const double kbend = 0.01e-3;
    const double kdamp = 0.2;
};
#endif
