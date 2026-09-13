#include "ParticleSystem.hpp"
#include "SpatialHash.hpp"
#include "SDF.hpp"
#include <glm/glm.hpp>
#include <random>
#include <cmath>

void ParticleSystem::initialize(uint32_t count, const glm::vec3& boundsMin, const glm::vec3& boundsMax, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> distX(boundsMin.x, boundsMax.x);
    std::uniform_real_distribution<float> distY(boundsMin.y, boundsMax.y);
    std::uniform_real_distribution<float> distZ(boundsMin.z, boundsMax.z);

    particles.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
        particles[i].position = glm::vec3(distX(rng), distY(rng), distZ(rng));
        particles[i].velocity = glm::vec3(0.0f);
        particles[i].normal = glm::vec3(0.0f, 1.0f, 0.0f);
        particles[i].projectionFrom = particles[i].position;
        particles[i].id = i;
    }
    // Partikelanzahl darf hier schrumpfen: alte Nachbarpaare wuerden sonst
    // auf gueltig erzeugte Indizes (particles[i]) zeigen -> Out-of-Bounds.
    m_spatialHash.clear();
}

bool ParticleSystem::projectToSDF(const SDF& sdf) {
    bool allOk = true;
    for (auto& p : particles) {
        p.projectionFrom = p.position;
        for (int iter = 0; iter < parameters.projectionIterations; ++iter) {
            SDFSample s = sdf.sample(p.position);
            float g2 = glm::dot(s.gradient, s.gradient);
            if (g2 < 1e-10f) {
                allOk = false;
                break;
            }
            p.position -= s.distance * s.gradient / g2;
            if (std::abs(s.distance) < parameters.sdfTolerance)
                break;
        }
        p.normal = glm::normalize(sdf.sample(p.position).gradient);
    }
    return allOk;
}

void ParticleSystem::buildSpatialHash() {
    std::vector<glm::vec3> positions;
    positions.reserve(particles.size());
    for (const auto& p : particles)
        positions.push_back(p.position);
    m_spatialHash.build(positions, parameters.repulsionRadius);
}

void ParticleSystem::relax(float dt, const SDF& sdf) {
    float stepDt = dt / std::max(1, parameters.substeps);
    float R = parameters.repulsionRadius;
    float k = parameters.repulsionStrength;
    constexpr float epsilon = 1e-6f;

    for (int sub = 0; sub < parameters.substeps; ++sub) {
        for (auto& p : particles)
            p.velocity *= 0.0f;

        for (auto& pair : m_spatialHash.pairs()) {
            auto& pi = particles[pair.i];
            auto& pj = particles[pair.j];
            glm::vec3 diff = pj.position - pi.position;
            float d = glm::length(diff);
            if (d < epsilon || d >= R) continue;

            float w = k * (1.0f - d / R) * (1.0f - d / R) / d;
            glm::vec3 force = -w * (diff / d); // stößt pi von pj ab

            pi.velocity += force;
            pj.velocity -= force;
        }

        for (auto& p : particles) {
            glm::vec3 tangentForce = p.velocity - glm::dot(p.velocity, p.normal) * p.normal;
            p.velocity = parameters.damping * tangentForce;

            glm::vec3 displacement = stepDt * p.velocity;
            float maxStep = parameters.maxStepLength * parameters.targetSpacing;
            float len = glm::length(displacement);
            if (len > maxStep)
                displacement *= maxStep / len;

            p.position += displacement;
        }

        projectToSDF(sdf);
        buildSpatialHash();
    }
}
