#include "cloth.h"
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;
using namespace glm;
using namespace Eigen;

Cloth::Cloth(const string &nodeFilename, const string &eleFilename,
        double scale, Vector3d startPos) {
    ifstream nodeIfs(nodeFilename.c_str());
    if (!nodeIfs) { cout << "dumlaya!" << endl; exit(1); }

    int naeem; // dummy variable
    int numVerts;
    nodeIfs >> numVerts >> naeem >> naeem >> naeem;
    /* cout << "numVerts: " << numVerts << endl; */

    cout << "scale: " << scale << endl;
    Pos.resize(numVerts, 3);
    double a, b;
    for (int i = 0; i < numVerts; i++) {
        nodeIfs >> naeem >> a >> b >> naeem;
        Pos(i, 0) = (a+1+startPos[0]) * scale / 2.0;
        Pos(i, 1) = startPos[2];
        Pos(i, 2) = (b+1+startPos[1]) * scale / 2.0;
    }

    ifstream eleIfs(eleFilename.c_str());
    int numFaces;
    eleIfs >> numFaces >> naeem >> naeem;
    /* cout << "numFaces: " << numFaces << endl; */
    F.resize(numFaces, 3);
    int x, y, z;
    for (int i = 0; i < numFaces; i++) {
        eleIfs >> naeem >> x >> y >> z;
        F.row(i) = Vector3i(x-1, y-1, z-1);
        Vector3d norm = (Pos.row(x-1)-Pos.row(y-1)).cross(Pos.row(y-1)-Pos.row(z-1));
        if (norm.dot(Vector3d(0, 1, 0)) < 0) {
            F.row(i) = Vector3i(x-1, z-1, y-1);
        }
        /* cout << norm.dot(Vector3d(0,1,0)) << endl << endl; */
        /* F.row(i) = Vector3i(x, y, z); */
    }
    oldPos.setZero();
}

Cloth::~Cloth() { }

void Cloth::generate_geometry(vector<glm::vec4>& obj_vertices,
        vector<glm::uvec3>& obj_faces) const {
    obj_vertices.clear();
    obj_faces.clear();
    for (uint i = 0; i < Pos.rows(); i++) {
        Vector3d row = Pos.row(i);
        /* cout << "row: " << row << endl; */
        obj_vertices.push_back(glm::vec4(row[0], row[1], row[2], 1.0));
    }
    for (uint i = 0; i < F.rows(); i++) {
        Vector3i face = F.row(i);
        /* cout << "face: " << face << endl; */
        obj_faces.push_back(glm::uvec3(face[0], face[1], face[2]));
    }
    /* cout << "obj_vertices.length(): " << obj_vertices.size() << endl; */
    /* cout << "obj_faces.length(): " << obj_faces.size() << endl; */
}
