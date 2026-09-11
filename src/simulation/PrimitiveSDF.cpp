#include "PrimitiveSDF.hpp"
#include <glm/glm.hpp>

SphereSDF::SphereSDF(glm::vec3 center, float radius)
    : m_center(center), m_radius(radius) {}

SDFSample SphereSDF::sample(glm::vec3 p) const {
    glm::vec3 diff = p - m_center;
    float dist = glm::length(diff) - m_radius;
    glm::vec3 grad = glm::normalize(diff);
    return {dist, grad};
}

glm::vec3 SphereSDF::boundsMin() const {
    return m_center - glm::vec3(m_radius);
}

glm::vec3 SphereSDF::boundsMax() const {
    return m_center + glm::vec3(m_radius);
}
