#pragma once

#include "SDF.hpp"

class SphereSDF : public SDF {
public:
    SphereSDF(glm::vec3 center, float radius);

    SDFSample sample(glm::vec3 p) const override;
    glm::vec3 boundsMin() const override;
    glm::vec3 boundsMax() const override;

private:
    glm::vec3 m_center;
    float m_radius;
};
