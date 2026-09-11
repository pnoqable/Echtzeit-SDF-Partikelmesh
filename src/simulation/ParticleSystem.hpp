#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

struct Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 normal;
    uint32_t id;
};

struct Triangle {
    uint32_t i0, i1, i2;
};

struct SimulationParameters {
    float targetSpacing = 0.1f;
    float repulsionRadius = 0.15f;
    float repulsionStrength = 1.0f;
    float damping = 0.9f;
    float maxStepLength = 0.01f;
    float sdfTolerance = 1e-4f;
    int substeps = 3;
    int projectionIterations = 10;
};

class ParticleSystem {
public:
    std::vector<Particle> particles;
    std::vector<Triangle> triangles;
    SimulationParameters parameters;

    void initialize(uint32_t count, const glm::vec3& boundsMin, const glm::vec3& boundsMax, uint32_t seed);
    bool projectToSDF(const class SDF& sdf);
};
