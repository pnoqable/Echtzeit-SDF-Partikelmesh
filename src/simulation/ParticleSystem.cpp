#include "ParticleSystem.hpp"
#include "SDF.hpp"
#include <glm/glm.hpp>
#include <random>

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
        particles[i].id = i;
    }
}

bool ParticleSystem::projectToSDF(const SDF& sdf) {
    bool allOk = true;
    for (auto& p : particles) {
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
