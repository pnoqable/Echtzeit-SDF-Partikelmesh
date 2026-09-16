#include "ParticleSystem.hpp"
#include "SpatialHash.hpp"
#include "SDF.hpp"
#include "../core/Profiler.hpp"
#include "../core/SIMD.hpp"
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
    auto _t = prof::Profiler::instance().scoped("project");
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
    auto _t = prof::Profiler::instance().scoped("grid");
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
        buildSpatialHash();

        // Kraft-Akkumulation (partikelparallel, paarweise via SIMD-Lanes).
        // Der Innenkern ist umgestellt auf d² statt d: Nur ein rsqrt-Wurzelzug
        // und eine Division pro 4er-Batch; der in-<R-Filter laeuft als
        // Quadratvergleich (kein sqrt fuer ausserhalb liegende Paare).
        {
            auto _t = prof::Profiler::instance().scoped("forces");
            const float R2 = R * R;
            const float eps2 = epsilon * epsilon;
            const float invR = 1.0f / R;
            const auto kk = simd::F4::set1(k);
            const auto oneF = simd::F4::set1(1.0f);
            const auto zeroF = simd::F4::zero();
            const auto eps2v = simd::F4::set1(eps2);
            const auto R2v = simd::F4::set1(R2);
            const auto invRv = simd::F4::set1(invR);

            m_pool.parallelFor(N, [&](std::size_t i) {
            auto& pi = particles[i];
            auto ck = m_spatialHash.cellOf(pi.position);
            const auto px = simd::F4::set1(pi.position.x);
            const auto py = simd::F4::set1(pi.position.y);
            const auto pz = simd::F4::set1(pi.position.z);
            simd::F4 ax = zeroF, ay = zeroF, az = zeroF;

            for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
                const auto* ids = m_spatialHash.idsInCell({ck.x + dx, ck.y + dy, ck.z + dz});
                if (!ids) continue;
                const std::size_t n = ids->size();
                // Restlaenge im letzten Batch wird mit dem eigenen Index
                // aufgefuellt; diff = 0 -> d2 unter eps² -> maskiert zu 0.
                for (std::size_t o = 0; o < n; o += 4) {
                    const std::size_t j0 = o < n ? (*ids)[o] : i;
                    const std::size_t j1 = o + 1 < n ? (*ids)[o + 1] : i;
                    const std::size_t j2 = o + 2 < n ? (*ids)[o + 2] : i;
                    const std::size_t j3 = o + 3 < n ? (*ids)[o + 3] : i;
                    auto loadX = [&](std::size_t j) { return particles[j].position.x; };
                    auto loadY = [&](std::size_t j) { return particles[j].position.y; };
                    auto loadZ = [&](std::size_t j) { return particles[j].position.z; };
                    float jx[4] = { loadX(j0), loadX(j1), loadX(j2), loadX(j3) };
                    float jy[4] = { loadY(j0), loadY(j1), loadY(j2), loadY(j3) };
                    float jz[4] = { loadZ(j0), loadZ(j1), loadZ(j2), loadZ(j3) };

                    const auto ddx = simd::F4::load4(jx) - px;
                    const auto ddy = simd::F4::load4(jy) - py;
                    const auto ddz = simd::F4::load4(jz) - pz;
                    const auto d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                    const auto inRange = simd::F4::andMask(
                        simd::F4::ge(d2, eps2v), simd::F4::lt(d2, R2v));
                    const auto d = simd::F4::sqrtv(d2);
                    const auto x = oneF - d * invRv;
                    const auto g = simd::F4::divv(kk * x * x, d2);
                    ax = ax + (g * ddx).neg().select(inRange, zeroF);
                    ay = ay + (g * ddy).neg().select(inRange, zeroF);
                    az = az + (g * ddz).neg().select(inRange, zeroF);
                }
            }
            pi.velocity *= parameters.damping;
            pi.velocity += glm::vec3(ax.hsum(), ay.hsum(), az.hsum());
            });
        }

        // Positionsintegration (partikelparallel)
        {
            auto _t = prof::Profiler::instance().scoped("integrate");
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
        }

        projectToSDF(sdf);
    }
}
