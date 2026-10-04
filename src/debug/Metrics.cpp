#include "Metrics.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace debug {

namespace {

constexpr float kUnderRatio = 0.85f;
constexpr float kOverRatio  = 1.15f;
constexpr float kMinAngleDeg = 20.0f;
constexpr float kMaxAspect    = 4.0f;

float angleDeg(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 ab = glm::normalize(b - a);
    glm::vec3 ac = glm::normalize(c - a);
    float d = glm::clamp(glm::dot(ab, ac), -1.0f, 1.0f);
    return std::acos(d) * 180.0f / glm::pi<float>();
}

} // namespace

std::vector<float> nearestDistances(const ParticleSystem& system) {
    const auto& particles = system.particles;
    const auto& hash = system.spatialHash();
    std::vector<float> nearest(particles.size(), std::numeric_limits<float>::max());

    // Partikelparallel ueber die 27 Nachbarzellen der eigenen Zelle, statt
    // paarweise ueber eine globale Paarliste: fuer jeden Partikel das Minimum
    // ueber dieselbe Nachbarschaft, die auch die Kraftberechnung in
    // ParticleSystem::relax() traversiert. Die alte Paarliste enthielt nur
    // Paare (i, j) mit j > i; da der Abstand symmetrisch ist, liefert die
    // Minimum-Formulierung pro Partikel exakt denselben Wert. Der Preis ist
    // ein doppelter Distanzabruf je Paar, der Gewinn entfaellt komplett:
    // keine Paarliste, kein Doppelpuffer pro Worker im build().
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const glm::vec3 pi = particles[i].position;
        const auto ck = hash.cellOf(pi);
        float best = std::numeric_limits<float>::max();
        for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
            const auto ids = hash.idsInCell({ck.x + dx, ck.y + dy, ck.z + dz});
            for (uint32_t k = 0; k < ids.count; ++k) {
                const std::size_t j = ids.data[k];
                if (j == i) continue; // eigener Partikel: d == 0 sonst immer
                best = std::min(best, glm::length(particles[j].position - pi));
            }
        }
        nearest[i] = best;
    }
    return nearest;
}

std::vector<float> spacingHistogram(const std::vector<float>& nearest, float targetSpacing, int bins, float maxDistRatio) {
    std::vector<float> result(static_cast<size_t>(bins), 0.0f);
    if (bins <= 0 || targetSpacing <= 0.0f) return result;
    float binWidth = maxDistRatio * targetSpacing / bins;
    for (float d : nearest) {
        if (d >= std::numeric_limits<float>::max()) continue;
        int idx = static_cast<int>(d / binWidth);
        if (idx >= bins) idx = bins - 1;
        if (idx >= 0) result[idx] += 1.0f;
    }
    return result;
}

SimulationMetrics evaluate(const ParticleSystem& system, const SDF& sdf,
                            float targetSpacing, const std::vector<float>& nearest) {
    SimulationMetrics m;

    double sum = 0.0, sumSq = 0.0;
    int valid = 0;
    m.distribution.minDist = std::numeric_limits<float>::max();
    for (const float d : nearest) {
        if (d >= std::numeric_limits<float>::max()) continue;
        ++valid;
        sum += d;
        sumSq += static_cast<double>(d) * d;
        m.distribution.minDist = std::min(m.distribution.minDist, d);
        m.distribution.maxDist = std::max(m.distribution.maxDist, d);

        float ratio = d / targetSpacing;
        if (ratio < kUnderRatio) ++m.distribution.underCount;
        else if (ratio > kOverRatio) ++m.distribution.overCount;
        else ++m.distribution.okCount;
    }
    if (valid > 0) {
        m.distribution.avgDist = static_cast<float>(sum / valid);
        m.distribution.stdDev = static_cast<float>(std::sqrt(std::max(0.0, sumSq / valid -
            m.distribution.avgDist * static_cast<double>(m.distribution.avgDist))));
    }

    double sumPhi = 0.0, sumSpeed = 0.0;
    for (const auto& p : system.particles) {
        float phi = std::abs(sdf.sample(p.position).distance);
        sumPhi += phi;
        m.sdf.maxAbsPhi = std::max(m.sdf.maxAbsPhi, phi);
        float sp = glm::length(p.velocity);
        sumSpeed += sp;
        m.maxSpeed = std::max(m.maxSpeed, sp);
    }
    if (!system.particles.empty()) {
        m.sdf.avgAbsPhi = static_cast<float>(sumPhi / system.particles.size());
        m.avgSpeed = static_cast<float>(sumSpeed / system.particles.size());
    }

    std::vector<glm::vec3> pos;
    pos.reserve(system.particles.size());
    for (const auto& p : system.particles) pos.push_back(p.position);

    for (const auto& t : system.triangles) {
        if (t.i0 >= pos.size() || t.i1 >= pos.size() || t.i2 >= pos.size()) {
            ++m.mesh.poorTriangles;
            continue;
        }
        const glm::vec3& a = pos[t.i0];
        const glm::vec3& b = pos[t.i1];
        const glm::vec3& c = pos[t.i2];

        float l0 = glm::length(b - a);
        float l1 = glm::length(c - b);
        float l2 = glm::length(a - c);
        float aspect = std::max({l0, l1, l2}) / std::max(1e-9f, std::min({l0, l1, l2}));
        float minDeg = std::min({angleDeg(a, b, c), angleDeg(b, a, c), angleDeg(c, a, b)});

        m.mesh.maxAspectRatio = std::max(m.mesh.maxAspectRatio, aspect);
        m.mesh.minAngleDeg = std::min(m.mesh.minAngleDeg, minDeg);
        if (minDeg < kMinAngleDeg || aspect > kMaxAspect) ++m.mesh.poorTriangles;
    }

    return m;
}

} // namespace debug