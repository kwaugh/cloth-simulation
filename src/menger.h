#ifndef MENGER_H
#define MENGER_H

#include <glm/glm.hpp>
#include <vector>

class Menger {
public:
	Menger();
	~Menger();
	void set_nesting_level(int);
	bool is_dirty() const;
	void set_clean();
	void generate_geometry(std::vector<glm::vec4>&,
            std::vector<glm::uvec3>&) const;
    void draw_cube(std::vector<glm::vec4>&,
            std::vector<glm::uvec3>&, glm::dvec3, glm::dvec3, int) const;
    void draw_inverse_cube(std::vector<glm::vec4>&,
            std::vector<glm::uvec3>&, glm::dvec3, glm::dvec3, int) const;
    void draw_pyramid(std::vector<glm::vec4>& obj_vertices,
            std::vector<glm::uvec3>& obj_faces,
            glm::dvec3 a,
            glm::dvec3 b,
            glm::dvec3 c,
            glm::dvec3 d,
            int depth) const;
    void iterate_mode(); 
private:
	int nesting_level_ = 0;
	bool dirty_ = false;
    int mode = 0; 
};

#endif
