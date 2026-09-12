#include "Triangulation.hpp"
#include "../simulation/SDF.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
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
    float targetSpacing, const SDF& sdf, const Parameters& params,
    MeshStats& stats
) const {
    glm::vec3 d = pb - pa;
    float len = glm::length(d);

    if (len > params.maxEdgeLength * targetSpacing) {
        stats.rejectedLength++;
        return false;
    }
    if (glm::dot(na, nb) < params.normalThreshold) {
        stats.rejectedNormal++;
        return false;
    }

    glm::vec3 mid = 0.5f * (pa + pb);
    float midDist = std::abs(sdf.sample(mid).distance);
    if (midDist > params.edgeMidpointTolerance * targetSpacing) {
        stats.rejectedMidpoint++;
        return false;
    }

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
            if (!acceptEdge(p, positions[qi], n, normals[qi], targetSpacing, sdf, params, m_stats)) continue;

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
            m_stats.rejectedManifold++;
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

    closeBoundaryLoops(positions, normals);
}

bool Triangulation::closeBoundaryLoops(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals
) {
    auto edgeKey = [](uint32_t a, uint32_t b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | b;
    };
    auto countEdges = [&]() {
        std::unordered_map<uint64_t, int> cnt;
        for (auto& t : m_triangles) {
            cnt[edgeKey(t.i0, t.i1)]++;
            cnt[edgeKey(t.i1, t.i2)]++;
            cnt[edgeKey(t.i2, t.i0)]++;
        }
        return cnt;
    };
    auto m0 = countEdges();

    // Randkanten-Adjazenz: jeder Randvertex hat Grad 2 -> geschlossene Loops.
    std::map<uint32_t, std::vector<uint32_t>> adj;
    for (auto& [k, c] : m0) {
        if (c == 1) {
            uint32_t a = static_cast<uint32_t>(k >> 32);
            uint32_t b = static_cast<uint32_t>(k);
            adj[a].push_back(b);
            adj[b].push_back(a);
        }
    }
    if (adj.empty()) return false;

    std::vector<std::vector<uint32_t>> loops;
    std::map<uint32_t, int> used;
    for (auto& [v, _] : adj) used[v] = 0;
    for (auto& [start, _] : adj) {
        if (used[start]) continue;
        std::vector<uint32_t> verts{ start };
        used[start] = 1;
        uint32_t cur = start;
        while (true) {
            auto& nb = adj[cur];
            uint32_t nxt = static_cast<uint32_t>(-1);
            for (auto w : nb) { if (!used[w]) { nxt = w; break; } }
            if (nxt == static_cast<uint32_t>(-1)) {
                for (auto w : nb) if (w == start) nxt = start;
            }
            if (nxt == static_cast<uint32_t>(-1)) break;
            if (nxt == start) { verts.push_back(start); break; }
            used[nxt] = 1;
            verts.push_back(nxt);
            cur = nxt;
            if (verts.size() > m0.size() + 2) break;
        }
        loops.push_back(std::move(verts));
    }

    bool filledAny = false;
    for (auto& verts : loops) {
        int n = static_cast<int>(verts.size()) - 1; // Knoten im Ring
        if (n != 3 && n != 4) continue;             // nur kleine Loops reparieren

        auto okOrientation = [&](uint32_t a, uint32_t b, uint32_t c) {
            glm::vec3 fn = glm::cross(positions[b] - positions[a], positions[c] - positions[a]);
            if (glm::length(fn) < 1e-8f) return false;
            glm::vec3 nmean = glm::normalize(normals[a] + normals[b] + normals[c]);
            return glm::dot(fn, nmean) > 0.0f;
        };

        bool good = true;
        std::vector<Triangle> newTris;
        if (n == 3) {
            uint32_t x = verts[0], y = verts[1], z = verts[2];
            if (!okOrientation(x, y, z)) std::swap(y, z);
            if (!okOrientation(x, y, z)) continue;
            if (m0[edgeKey(x, y)] >= 2 || m0[edgeKey(y, z)] >= 2 || m0[edgeKey(z, x)] >= 2) continue;
            newTris.push_back({ x, y, z });
        } else { // n == 4
            uint32_t v0 = verts[0], v1 = verts[1], v2idx = verts[2], v3 = verts[3];
            bool useD02 = glm::length(positions[v2idx] - positions[v0])
                <= glm::length(positions[v3] - positions[v1]);
            std::vector<Triangle> cand;
            if (useD02) {
                cand = { {v0, v1, v2idx}, {v0, v2idx, v3} };
            } else {
                cand = { {v1, v2idx, v3}, {v1, v3, v0} };
            }
            for (auto& t : cand) {
                uint32_t x = t.i0, y = t.i1, z = t.i2;
                if (!okOrientation(x, y, z)) std::swap(y, z);
                if (!okOrientation(x, y, z)) { good = false; break; }
                if (m0[edgeKey(x, y)] >= 2 || m0[edgeKey(y, z)] >= 2 || m0[edgeKey(z, x)] >= 2) { good = false; break; }
            }
            if (!good) continue;
            newTris = cand;
        }

        for (auto& t : newTris) {
            m_triangles.push_back(t);
            m0[edgeKey(t.i0, t.i1)]++;
            m0[edgeKey(t.i1, t.i2)]++;
            m0[edgeKey(t.i2, t.i0)]++;
            m_stats.totalTriangles++;
        }
        filledAny = true;
    }
    return filledAny;
}