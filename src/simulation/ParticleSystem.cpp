#include "ParticleSystem.hpp"
#include "SpatialHash.hpp"
#include "SDF.hpp"
#include <glm/glm.hpp>
#include <atomic>
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

// Partikelparallel: jeder Particle schreibt NUR seine eigenen Felder;
// damit gibt es keine Daten-Races zwischen Threads.
bool ParticleSystem::projectToSDF(const SDF& sdf) {
    std::atomic<int> fails{0};
    const std::size_t N = particles.size();
    m_pool.parallelFor(N, [&](std::size_t i) {
        auto& p = particles[i];
        p.projectionFrom = p.position;
        bool ok = true;
        for (int iter = 0; iter < parameters.projectionIterations; ++iter) {
            SDFSample s = sdf.sample(p.position);
            float g2 = glm::dot(s.gradient, s.gradient);
            if (g2 < 1e-10f) { ok = false; break; }
            p.position -= s.distance * s.gradient / g2;
            if (std::abs(s.distance) < parameters.sdfTolerance)
                break;
        }
        p.normal = glm::normalize(sdf.sample(p.position).gradient);
        if (!ok) ++fails;
    });
    return fails.load() == 0;
}

void ParticleSystem::buildSpatialHash() {
    std::vector<glm::vec3> positions;
    positions.reserve(particles.size());
    for (const auto& p : particles)
        positions.push_back(p.position);
    m_spatialHash.build(positions, parameters.repulsionRadius, &m_pool);
}

// Partikelparallel statt Paar-parallel:
//   Jeder Thread iteriert selbst die 27 Nachbarzellen seiner Zelle und
//   summiert die Abstosskraefte in ein locales acc. velocity[i] = acc
//   (kein += noetig, da Zustaende vorher zurueckgesetzt werden).
//   Vorteil: kein Race auf velocity[i] oder velocity[j]; jede
//   Nachbarzuordnung zweimal besucht (einmal pro Endpoint) statt einmal,
//   aber dafuer braucht es weder Reduce-Puffer noch Atomics.
void ParticleSystem::relax(float dt, const SDF& sdf) {
    float stepDt = dt / std::max(1, parameters.substeps);
    float R = parameters.repulsionRadius;
    float k = parameters.repulsionStrength;
    constexpr float epsilon = 1e-6f;
    const std::size_t N = particles.size();

    for (int sub = 0; sub < parameters.substeps; ++sub) {
        // Kraft-Akkumulation (partikelparallel)
        m_pool.parallelFor(N, [&](std::size_t i) {
            auto& pi = particles[i];
            glm::vec3 acc(0.0f);
            auto ck = m_spatialHash.cellOf(pi.position);

            for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
                const auto* ids = m_spatialHash.idsInCell({ck.x + dx, ck.y + dy, ck.z + dz});
                if (!ids) continue;
                for (uint32_t j : *ids) {
                    if (j == i) continue;
                    const auto& pj = particles[j];
                    glm::vec3 diff = pj.position - pi.position;
                    float d = glm::length(diff);
                    if (d < epsilon || d >= R) continue;
                    float x = 1.0f - d / R;
                    float w = k * x * x / d;
                    acc += -w * (diff / d);
                }
            }
            pi.velocity *= parameters.damping;
            pi.velocity += acc;
        });

        // Positionsintegration (partikelparallel)
        m_pool.parallelFor(N, [&](std::size_t i) {
            auto& p = particles[i];
            p.velocity = p.velocity - glm::dot(p.velocity, p.normal) * p.normal;

            glm::vec3 displacement = stepDt * p.velocity;
            float maxStep = parameters.maxStepLength * parameters.targetSpacing;
            float len = glm::length(displacement);
            if (len > maxStep)
                displacement *= maxStep / len;

            p.position += displacement;
        });

        projectToSDF(sdf);
        buildSpatialHash();
    }
}
