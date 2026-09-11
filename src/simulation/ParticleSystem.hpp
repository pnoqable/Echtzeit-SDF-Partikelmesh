#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include "../mesh/Triangulation.hpp"
#include "SpatialHash.hpp"

struct Particle {
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 normal;
    uint32_t id;
};

struct SimulationParameters {
    float targetSpacing = 0.1f;
    float repulsionRadius = 0.15f;
    float repulsionStrength = 1.0f;
    float damping = 0.9f;
    float maxStepLength = 0.2f;
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
    void buildSpatialHash();
    void relax(float dt, const SDF& sdf);

    class SpatialHash& spatialHash() { return m_spatialHash; }
    const class SpatialHash& spatialHash() const { return m_spatialHash; }

private:
    class SpatialHash m_spatialHash;
};
