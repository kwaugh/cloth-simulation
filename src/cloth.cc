#include "cloth.h"
#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace glm;

Cloth::Cloth() { }

Cloth::~Cloth() { }

// FIXME generate Cloth sponge geometry
void Cloth::generate_geometry(vector<vec4>& obj_vertices,
        vector<uvec3>& obj_faces) const {
    obj_vertices.push_back(vec4(1, 1, 1, 1));
    obj_vertices.push_back(vec4(1, -1, -1, 1));
    obj_vertices.push_back(vec4(-1, 1, -1, 1));
    obj_vertices.push_back(vec4(-1, -1, 1, 1));

    obj_faces.push_back(uvec3(0, 2, 1));
    //obj_faces.push_back(uvec3(0, 3, 2));
    //obj_faces.push_back(uvec3(0, 1, 3));
    //obj_faces.push_back(uvec3(1, 2, 3));
}
