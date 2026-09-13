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
    // Nur die Kantenlänge begrenzt den Delaunay-Ring. Normalen- und
    // Midpoint-Kriterien wurden entfernt: Die relaxierte Partikelverteilung
    // garantiert, dass Sehnen der Oberfläche folgen (Kantenmitten ~0.13h,
    // unter jeder sinnvollen Toleranz). An konkaven Nahten (Hantel) verwarfen
    // beide Kriterien stattdessen geometrisch valide Kanten und erzeugten Loecher.
    (void)na; (void)nb; (void)sdf; (void)stats;
    glm::vec3 d = pb - pa;
    return glm::length(d) <= params.maxEdgeLength * targetSpacing;
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

    // Delaunay-artige Priorisierung: Zuerst die raeumlich kompaktesten Kandidaten
    // (kürzeste längste Kante). Auf direktional gestreckten Flächen (Ellipsoid)
    // gewinnen damit die richtigen Dreiecke den Wettbewerb um geteilte Kanten,
    // statt dass willkürlich frühe Partikel den Manifold-Flaschenhals erzeugen.
    struct RankedTri {
        TriCandidate t;
        float longestEdge;
    };
    std::vector<RankedTri> ranked;
    ranked.reserve(candidates.size());
    for (auto& t : candidates) {
        float e0 = glm::length(positions[t.a] - positions[t.b]);
        float e1 = glm::length(positions[t.b] - positions[t.c]);
        float e2 = glm::length(positions[t.c] - positions[t.a]);
        ranked.push_back({ t, std::max(e0, std::max(e1, e2)) });
    }
    std::sort(ranked.begin(), ranked.end(),
        [](const RankedTri& a, const RankedTri& b) { return a.longestEdge < b.longestEdge; });

    for (auto& rt : ranked) {
        auto& t = rt.t;
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

    closeBoundaryLoops(positions, normals, targetSpacing, sdf, params);
}

bool Triangulation::closeBoundaryLoops(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals,
    float targetSpacing,
    const SDF& sdf,
    const Parameters& params
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
        if (n < 3) continue;

        auto okOrientation = [&](uint32_t a, uint32_t b, uint32_t c) {
            glm::vec3 fn = glm::cross(positions[b] - positions[a], positions[c] - positions[a]);
            if (glm::length(fn) < 1e-8f) return false;
            glm::vec3 nmean = glm::normalize(normals[a] + normals[b] + normals[c]);
            return glm::dot(fn, nmean) > 0.0f;
        };
        auto okManifold = [&](uint32_t a, uint32_t b) {
            return m0[edgeKey(a, b)] < 2;
        };
        // Loop-Diagonale darf nicht quer durchs Volumen laufen: Sehnen-
        // mitte muss nah an der Oberflaeche bleiben (0.5 h). Grosszuegiger
        // als der alte 0.15-h-Kantenfilter, damit die konkave Hantel-Naht
        // (Diagonalen liegen 0.13..0.18 h ab) noch schliesst, waehrend
        // Diagonale, deren Sehne quer durch die Form fuehrt, offen bleiben.
        auto okDiag = [&](uint32_t a, uint32_t b) {
            glm::vec3 mid = 0.5f * (positions[a] + positions[b]);
            return std::abs(sdf.sample(mid).distance) <= 0.5f * targetSpacing;
        };

        // Ear-Clipping fuer beliebige Loop-Groessen. An der Hantel-Naht
        // (konkaver Knick, 13..25 Knoten) schliesst es die zwei grossen
        // Randloecher, die der 3er/4er-Loop-Fill bisher offen liess.
        // Beim Clipping wird jeweils das Ohr mit der kuerzesten Diagonale
        // entfernt (kompakte Dreiecke), solange okDiag erfuellt ist.
        std::vector<uint32_t> ring;
        ring.reserve(n);
        for (int i = 0; i < n; ++i) ring.push_back(verts[i]);

        std::vector<Triangle> newTris;
        while (ring.size() > 3) {
            size_t m = ring.size();
            size_t bestI = 0;
            float bestDiag = 1e30f;
            for (size_t i = 0; i < m; ++i) {
                uint32_t prev = ring[(i + m - 1) % m];
                uint32_t cur  = ring[i];
                uint32_t next = ring[(i + 1) % m];
                if (prev == cur || cur == next || prev == next) continue;
                float diag = glm::length(positions[prev] - positions[next]);
                if (diag >= bestDiag) continue;
                if (!okManifold(prev, next)) continue;
                if (!okDiag(prev, next)) continue;
                if (!okOrientation(prev, cur, next)) continue;
                bestI = i;
                bestDiag = diag;
            }
            if (bestDiag >= 1e30f) {  // kein Ohr mehr abnehmbar
                newTris.clear();
                break;
            }
            size_t i = bestI;
            uint32_t prev = ring[(i + m - 1) % m];
            uint32_t cur  = ring[i];
            uint32_t next = ring[(i + 1) % m];
            newTris.push_back({ prev, cur, next });
            ring.erase(ring.begin() + i);
        }
        if (ring.size() == 3 && !newTris.empty()) {
            uint32_t x = ring[0], y = ring[1], z = ring[2];
            if (okManifold(x, y) && okManifold(y, z) && okManifold(z, x)) {
                uint32_t yy = y, zz = z;
                if (!okOrientation(x, yy, zz)) std::swap(yy, zz);
                if (okOrientation(x, yy, zz)) newTris.push_back({ x, yy, zz });
            }
        }

        if (newTris.empty()) continue;
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