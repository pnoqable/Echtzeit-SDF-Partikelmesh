#include "Triangulation.hpp"
#include "../simulation/SDF.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

struct EdgeTriangHash {
    size_t operator()(const Edge& e) const {
        return std::hash<uint32_t>()(e.a) ^ (std::hash<uint32_t>()(e.b) << 1);
    }
};

TangentBasis Triangulation::computeTangentBasis(glm::vec3 normal) const {
    glm::vec3 up = std::abs(normal.y) < 0.99f
        ? glm::vec3(0, 1, 0)
        : glm::vec3(1, 0, 0);
    glm::vec3 u = glm::normalize(glm::cross(up, normal));
    glm::vec3 v = glm::cross(normal, u);
    return {u, v};
}

bool Triangulation::acceptEdge(
    glm::vec3 pa, glm::vec3 pb,
    glm::vec3 na, glm::vec3 nb,
    float targetSpacing, const SDF& sdf, const Parameters& params
) const {
    glm::vec3 d = pb - pa;
    float len = glm::length(d);

    if (len > params.maxEdgeLength * targetSpacing) return false;
    if (glm::dot(na, nb) < params.normalThreshold) return false;

    glm::vec3 mid = 0.5f * (pa + pb);
    float midDist = std::abs(sdf.sample(mid).distance);
    if (midDist > params.edgeMidpointTolerance * targetSpacing) return false;

    return true;
}

void Triangulation::build(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals,
    float targetSpacing,
    const SDF& sdf,
    const Parameters& params
) {
    m_triangles.clear();
    m_stats = {};

    size_t N = positions.size();
    if (N < 3) return;

    float R = params.maxEdgeLength * targetSpacing;

    // Kandidaten-Dreiecke als Fan um jeden Partikel erzeugen
    struct TriCandidate {
        uint32_t a, b, c;
    };
    std::vector<TriCandidate> candidates;

    for (uint32_t pi = 0; pi < N; ++pi) {
        glm::vec3 p = positions[pi];
        glm::vec3 n = normals[pi];
        TangentBasis basis = computeTangentBasis(n);

        struct Neighbor2D {
            uint32_t id;
            float angle;
        };
        std::vector<Neighbor2D> localNeighbors;

        for (uint32_t qi = 0; qi < N; ++qi) {
            if (qi == pi) continue;
            glm::vec3 d = positions[qi] - p;
            if (glm::length(d) > R) continue;
            if (!acceptEdge(p, positions[qi], n, normals[qi], targetSpacing, sdf, params)) continue;

            float x = glm::dot(d, basis.u);
            float y = glm::dot(d, basis.v);
            localNeighbors.push_back({qi, atan2f(y, x)});
        }

        if (localNeighbors.size() < 2) continue;

        std::sort(localNeighbors.begin(), localNeighbors.end(),
            [](const Neighbor2D& a, const Neighbor2D& b) { return a.angle < b.angle; });

        for (size_t k = 0; k < localNeighbors.size(); ++k) {
            size_t k2 = (k + 1) % localNeighbors.size();
            uint32_t a = localNeighbors[k].id;
            uint32_t b = localNeighbors[k2].id;

            glm::vec3 v0 = positions[a] - p;
            glm::vec3 v1 = positions[b] - p;
            glm::vec3 fn = glm::cross(v0, v1);
            if (glm::length(fn) < 1e-8f) { m_stats.degenerate++; continue; }

            // Orientierung vereinheitlichen
            uint32_t i0 = a, i1 = b;
            if (glm::dot(fn, n) < 0.0f) std::swap(i0, i1);

            candidates.push_back({pi, i0, i1});
        }
    }

    // Deduplizierung identischer Dreiecke (geteilte Flächen)
    auto triKey = [](TriCandidate t) {
        uint32_t mins[3] = {t.a, t.b, t.c};
        std::sort(mins, mins + 3);
        std::array<uint32_t, 3> key{mins[0], mins[1], mins[2]};
        return key;
    };
    struct TriKeyHash {
        size_t operator()(const std::array<uint32_t, 3>& k) const {
            return std::hash<uint32_t>()(k[0]) ^
                   (std::hash<uint32_t>()(k[1]) << 1) ^
                   (std::hash<uint32_t>()(k[2]) << 2);
        }
    };
    std::unordered_set<std::array<uint32_t, 3>, TriKeyHash> seen;

    struct EdgeCount {
        uint32_t count = 0;
    };
    std::unordered_map<Edge, EdgeCount, EdgeTriangHash> edgeCounts;
    m_triangles.clear();

    for (auto& t : candidates) {
        std::array<uint32_t, 3> key = triKey(t);
        if (seen.count(key)) continue;
        seen.insert(key);

        Edge e01{std::min(t.a, t.b), std::max(t.a, t.b)};
        Edge e12{std::min(t.b, t.c), std::max(t.b, t.c)};
        Edge e20{std::min(t.c, t.a), std::max(t.c, t.a)};
        if (e01.a == e01.b || e12.a == e12.b || e20.a == e20.b) {
            m_stats.degenerate++;
            continue;
        }

        // Kante höchstens 2x teilen (manifold: Rand oder 2 Nachbarn)
        if (edgeCounts[e01].count >= 2 ||
            edgeCounts[e12].count >= 2 ||
            edgeCounts[e20].count >= 2) {
            m_stats.rejectedMidpoint++;
            continue;
        }

        edgeCounts[e01].count++;
        edgeCounts[e12].count++;
        edgeCounts[e20].count++;
        m_triangles.push_back({t.a, t.b, t.c});
    }

    // Finale Validierung/Orientierung der aufgenommenen Dreiecke
    std::vector<Triangle> finalTriangles;
    finalTriangles.reserve(m_triangles.size());
    for (auto& tri : m_triangles) {
        glm::vec3 v0 = positions[tri.i1] - positions[tri.i0];
        glm::vec3 v1 = positions[tri.i2] - positions[tri.i0];
        glm::vec3 fn = glm::cross(v0, v1);
        float len = glm::length(fn);
        if (len < 1e-8f) { m_stats.degenerate++; continue; }
        fn /= len;

        glm::vec3 avgN = glm::normalize(
            normals[tri.i0] + normals[tri.i1] + normals[tri.i2]);
        if (glm::dot(fn, avgN) < 0.0f) { m_stats.wrongOrientation++; continue; }

        finalTriangles.push_back(tri);
    }

    m_triangles = std::move(finalTriangles);
    m_stats.totalTriangles = static_cast<int>(m_triangles.size());
}