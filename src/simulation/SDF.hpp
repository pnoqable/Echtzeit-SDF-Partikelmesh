#pragma once

#include <glm/glm.hpp>

struct SDFSample {
    float distance;
    glm::vec3 gradient;
};

class SDF {
public:
    virtual ~SDF() = default;
    virtual SDFSample sample(glm::vec3 p) const = 0;
    virtual glm::vec3 boundsMin() const = 0;
    virtual glm::vec3 boundsMax() const = 0;
};
