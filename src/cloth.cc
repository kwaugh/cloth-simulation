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
}
