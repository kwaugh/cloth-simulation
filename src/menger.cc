#include "menger.h"
#include <iostream>

namespace {
    const int kMinLevel = 0;
    const int kMaxLevel = 4;
};

Menger::Menger() {
    // Add additional initialization if you like
}

Menger::~Menger() {
}

void Menger::set_nesting_level(int level) {
    dirty_ = (level != nesting_level_);
    nesting_level_ = level;
}

bool Menger::is_dirty() const {
    return dirty_;
}

void Menger::set_clean() {
    dirty_ = false;
}

// FIXME generate Menger sponge geometry
void Menger::generate_geometry(std::vector<glm::vec4>& obj_vertices,
        std::vector<glm::uvec3>& obj_faces) const {
    
    if(mode == 0) {
        draw_cube(obj_vertices, obj_faces, glm::dvec3(-0.5), glm::dvec3(0.5), nesting_level_);
    } else if(mode == 1) {
        draw_pyramid(obj_vertices, obj_faces, glm::dvec3(1, 1, 1),  glm::dvec3(1, -1, -1), glm::dvec3(-1, 1, -1), glm::dvec3(-1, -1, 1), nesting_level_);
    } else if(mode == 2) {
        draw_inverse_cube(obj_vertices, obj_faces, glm::dvec3(-0.5), glm::dvec3(0.5), nesting_level_);
    }
}

void Menger::draw_pyramid(std::vector<glm::vec4>& obj_vertices,
        std::vector<glm::uvec3>& obj_faces,
        glm::dvec3 a,
        glm::dvec3 b,
        glm::dvec3 c,
        glm::dvec3 d,
        int depth) const {
    if(depth == -1) {
        uint sz = obj_vertices.size();
        obj_vertices.push_back(glm::vec4(a, 1.0));
        obj_vertices.push_back(glm::vec4(b, 1.0));
        obj_vertices.push_back(glm::vec4(c, 1.0));
        obj_vertices.push_back(glm::vec4(d, 1.0));

        obj_faces.push_back(sz + glm::uvec3(0, 2, 1));
        obj_faces.push_back(sz + glm::uvec3(0, 3, 2));
        obj_faces.push_back(sz + glm::uvec3(0, 1, 3));
        obj_faces.push_back(sz + glm::uvec3(1, 2, 3));
    } else {
        draw_pyramid(obj_vertices, obj_faces, 
                a,           (a+b) * 0.5, (a+c) * 0.5, (a+d) * 0.5, depth-1);
        draw_pyramid(obj_vertices, obj_faces, 
                (a+b) * 0.5, b,           (b+c) * 0.5, (b+d) * 0.5, depth-1);
        draw_pyramid(obj_vertices, obj_faces, 
                (a+c) * 0.5, (c+b) * 0.5, c,           (c+d) * 0.5, depth-1);
        draw_pyramid(obj_vertices, obj_faces, 
                (a+d) * 0.5, (d+b) * 0.5, (d+c) * 0.5, d,           depth-1);
    }
}

void Menger::draw_cube(std::vector<glm::vec4>& obj_vertices,
        std::vector<glm::uvec3>& obj_faces, glm::dvec3 minN, glm::dvec3 maxN, int depth) const {
    if (depth == -1) {
        uint sz = obj_vertices.size();
        obj_vertices.push_back(glm::vec4(minN[0], maxN[1], maxN[2], 1.0)); /* a */
        obj_vertices.push_back(glm::vec4(minN[0], maxN[1], minN[2], 1.0)); /* b */
        obj_vertices.push_back(glm::vec4(maxN[0], maxN[1], minN[2], 1.0)); /* c */
        obj_vertices.push_back(glm::vec4(maxN[0], maxN[1], maxN[2], 1.0)); /* d */
        obj_vertices.push_back(glm::vec4(minN[0], minN[1], maxN[2], 1.0)); /* e */
        obj_vertices.push_back(glm::vec4(minN[0], minN[1], minN[2], 1.0)); /* f */
        obj_vertices.push_back(glm::vec4(maxN[0], minN[1], minN[2], 1.0)); /* g */
        obj_vertices.push_back(glm::vec4(maxN[0], minN[1], maxN[2], 1.0)); /* h */


        obj_faces.push_back(sz + glm::uvec3(0, 2, 1));  /*top*/ 
        obj_faces.push_back(sz + glm::uvec3(0, 3, 2));
        obj_faces.push_back(sz + glm::uvec3(0, 4, 7)); /* front */
        obj_faces.push_back(sz + glm::uvec3(0, 7, 3));
        obj_faces.push_back(sz + glm::uvec3(3, 7, 6)); /* right */
        obj_faces.push_back(sz + glm::uvec3(3, 6, 2));
        obj_faces.push_back(sz + glm::uvec3(2, 6, 5)); /* back */
        obj_faces.push_back(sz + glm::uvec3(2, 5, 1));
        obj_faces.push_back(sz + glm::uvec3(1, 5, 4)); /* left */
        obj_faces.push_back(sz + glm::uvec3(1, 4, 0));
        obj_faces.push_back(sz + glm::uvec3(4, 6, 7)); /* bottom */
        obj_faces.push_back(sz + glm::uvec3(4, 5, 6));

    } else {
        for(uint i = 0; i < 3; i++) {
            for(uint j = 0; j < 3; j++) {
                for(uint k = 0; k < 3; k++) {
                    int num1s = (i == 1) ? 1 : 0;
                    num1s += (j == 1) ? 1 : 0;
                    num1s += (k == 1) ? 1 : 0;

                    if(num1s <= 1) {
                        double size = (maxN[0] - minN[0])/3.0;

                        glm::dvec3 nextMin(minN[0] + size*i    , minN[1] + size*j    , minN[2] + size*k    );
                        glm::dvec3 nextMax(minN[0] + size*(i+1), minN[1] + size*(j+1), minN[2] + size*(k+1));
                        draw_cube(obj_vertices, obj_faces, nextMin, nextMax, depth-1);

                    }
                }
            }
        }
    }
}

void Menger::draw_inverse_cube(std::vector<glm::vec4>& obj_vertices,
        std::vector<glm::uvec3>& obj_faces, glm::dvec3 minN, glm::dvec3 maxN, int depth) const {
    if (depth == -1) {
        uint sz = obj_vertices.size();
        obj_vertices.push_back(glm::vec4(minN[0], maxN[1], maxN[2], 1.0)); /* a */
        obj_vertices.push_back(glm::vec4(minN[0], maxN[1], minN[2], 1.0)); /* b */
        obj_vertices.push_back(glm::vec4(maxN[0], maxN[1], minN[2], 1.0)); /* c */
        obj_vertices.push_back(glm::vec4(maxN[0], maxN[1], maxN[2], 1.0)); /* d */
        obj_vertices.push_back(glm::vec4(minN[0], minN[1], maxN[2], 1.0)); /* e */
        obj_vertices.push_back(glm::vec4(minN[0], minN[1], minN[2], 1.0)); /* f */
        obj_vertices.push_back(glm::vec4(maxN[0], minN[1], minN[2], 1.0)); /* g */
        obj_vertices.push_back(glm::vec4(maxN[0], minN[1], maxN[2], 1.0)); /* h */


        obj_faces.push_back(sz + glm::uvec3(0, 2, 1));  /*top*/ 
        obj_faces.push_back(sz + glm::uvec3(0, 3, 2));
        obj_faces.push_back(sz + glm::uvec3(0, 4, 7)); /* front */
        obj_faces.push_back(sz + glm::uvec3(0, 7, 3));
        obj_faces.push_back(sz + glm::uvec3(3, 7, 6)); /* right */
        obj_faces.push_back(sz + glm::uvec3(3, 6, 2));
        obj_faces.push_back(sz + glm::uvec3(2, 6, 5)); /* back */
        obj_faces.push_back(sz + glm::uvec3(2, 5, 1));
        obj_faces.push_back(sz + glm::uvec3(1, 5, 4)); /* left */
        obj_faces.push_back(sz + glm::uvec3(1, 4, 0));
        obj_faces.push_back(sz + glm::uvec3(4, 6, 7)); /* bottom */
        obj_faces.push_back(sz + glm::uvec3(4, 5, 6));

    } else {
        for(uint i = 0; i < 3; i++) {
            for(uint j = 0; j < 3; j++) {
                for(uint k = 0; k < 3; k++) {
                    int num1s = (i == 1) ? 1 : 0;
                    num1s += (j == 1) ? 1 : 0;
                    num1s += (k == 1) ? 1 : 0;

                    if(num1s == 0) {
                        double size = (maxN[0] - minN[0])/3.0;

                        glm::dvec3 nextMin(minN[0] + size*i    , minN[1] + size*j    , minN[2] + size*k    );
                        glm::dvec3 nextMax(minN[0] + size*(i+1), minN[1] + size*(j+1), minN[2] + size*(k+1));
                        draw_cube(obj_vertices, obj_faces, nextMin, nextMax, depth-1);

                    }
                }
            }
        }
    }
}

void Menger::iterate_mode() {
    mode = (mode + 1) % 3;
    dirty_ = true;
}
