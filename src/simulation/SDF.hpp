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
    // Analytische Oberflaechenflaeche; Grundlage fuer das Partikel-Spacing
    // h = sqrt(A / N) (mittlere Punktdichte statt Hexagon-Ringabstand).
    virtual float surfaceArea() const = 0;
};
