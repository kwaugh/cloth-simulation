#ifndef MENGER_H
#define MENGER_H

#include <glm/glm.hpp>
#include <vector>

class Cloth {
public:
    Cloth();
    ~Cloth();
    void generate_geometry(std::vector<glm::vec4>&,
        std::vector<glm::uvec3>&) const;
private:
    
};

#endif
