#include "Triangulation.hpp"
#include "../simulation/SDF.hpp"
#include "../core/ThreadPool.hpp"
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

// Uniformer Grid passend zum Delaunay-Ring: Zellgroesse = R (der Suchradius).
// So liegen alle Kandidaten (|d| <= R) garantiert in den 3x3x3 Nachbarzellen
// um die Zelle des Partikels (eine Kante weiter wuerde |d| > R bedeuten, weil
// die Zelle selbst schon die halbe Ring-Hoehe ausmacht). Der Ring wird daher
// nur in den 27 umliegenden Zellen gesucht, nicht ueber alle N Partikel.
void Triangulation::GridBuilder::build(const std::vector<glm::vec3>& positions, float radius) {
    const std::size_t N = positions.size();
    cellSize = std::max(radius, 1e-6f);
    if (N == 0) return;

    glm::vec3 lo = positions[0], hi = positions[0];
    for (const auto& p : positions) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    // Eine Zelle Abstand als Padding, damit auch Outlier sicher landen.
    origin = { static_cast<int>(std::floor(lo.x / cellSize)) - 1,
               static_cast<int>(std::floor(lo.y / cellSize)) - 1,
               static_cast<int>(std::floor(lo.z / cellSize)) - 1 };
    dims = { static_cast<int>(std::floor(hi.x / cellSize)) - origin[0] + 2,
             static_cast<int>(std::floor(hi.y / cellSize)) - origin[1] + 2,
             static_cast<int>(std::floor(hi.z / cellSize)) - origin[2] + 2 };

    // Zellgroessen begrenzen (Sicherheit gegen ausserodentliche Skalierungen).
    for (int d = 0; d < 3; ++d) dims[d] = std::max(1, std::min(dims[d], 1 << 24));

    auto lin = [&](int x, int y, int z) {
        return (static_cast<std::size_t>(x) * dims[1] + y) * dims[2] + z;
    };

    // (ZellIndex, PartikelID) sortieren -> kompaktes Start/Ende je Zelle.
    std::vector<std::pair<std::size_t, uint32_t>> entries;
    entries.reserve(N);
    std::vector<std::array<int, 3>> cells(N);
    for (std::size_t i = 0; i < N; ++i) {
        const glm::vec3& p = positions[i];
        const int cx = static_cast<int>(std::floor(p.x / cellSize)) - origin[0];
        const int cy = static_cast<int>(std::floor(p.y / cellSize)) - origin[1];
        const int cz = static_cast<int>(std::floor(p.z / cellSize)) - origin[2];
        const int ccx = std::max(0, std::min(cx, dims[0] - 1));
        const int ccy = std::max(0, std::min(cy, dims[1] - 1));
        const int ccz = std::max(0, std::min(cz, dims[2] - 1));
        cells[i] = { ccx, ccy, ccz };
        entries.emplace_back(lin(ccx, ccy, ccz), static_cast<uint32_t>(i));
    }
    std::sort(entries.begin(), entries.end());

    m_cellIds.assign(entries.size(), 0);
    for (std::size_t i = 0; i < entries.size(); ++i)
        m_cellIds[i] = entries[i].second;

    // m_ranges: (start, ende+1) je Zelle, fuer nicht besetzte Zellen leer.
    const std::size_t totalCells = static_cast<std::size_t>(dims[0]) * dims[1] * dims[2];
    m_ranges.assign(totalCells, { 0, 0 });
    std::size_t begin = 0;
    while (begin < entries.size()) {
        const std::size_t cellIdx = entries[begin].first;
        std::size_t end = begin + 1;
        while (end < entries.size() && entries[end].first == cellIdx) ++end;
        m_ranges[cellIdx] = { static_cast<uint32_t>(begin), static_cast<uint32_t>(end) };
        begin = end;
    }
}

void Triangulation::GridBuilder::query(
    const std::array<int, 3>& cell, const std::vector<glm::vec3>& positions,
    float radius, std::vector<uint32_t>& out) const {
    (void)positions; (void)radius; // Suchradius ist == Zellgroesse, Ring = +-1 Zelle.
    const int cx = std::max(0, std::min(cell[0], dims[0] - 1));
    const int cy = std::max(0, std::min(cell[1], dims[1] - 1));
    const int cz = std::max(0, std::min(cell[2], dims[2] - 1));
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const int nx = cx + dx, ny = cy + dy, nz = cz + dz;
                if (nx < 0 || ny < 0 || nz < 0 || nx >= dims[0] || ny >= dims[1] || nz >= dims[2]) continue;
                const std::size_t idx = (static_cast<std::size_t>(nx) * dims[1] + ny) * dims[2] + nz;
                const auto& r = m_ranges[idx];
                for (uint32_t k = r.first; k < r.second; ++k)
                    out.push_back(m_cellIds[k]);
            }
        }
    }
}

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
    (void)na; (void)nb; (void)sdf;
    glm::vec3 d = pb - pa;
    if (glm::length(d) > params.maxEdgeLength * targetSpacing) {
        stats.rejectedLength++;
        return false;
    }
    return true;
}

void Triangulation::build(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals,
    float targetSpacing,
    const SDF& sdf,
    const Parameters& params,
    ThreadPool* pool
) {
    m_triangles.clear();
    m_stats = {};

    size_t N = positions.size();
    if (N < 3) return;

    // Uniformer Grid ueber den Delaunay-Ring (Zellgroesse = R). Die
    // Nachbarsuche pro Partikel fragt nur die 27 umliegenden Zellen ab
    // statt alle N Partikel: O(N * Zellring) statt O(N^2).
    const float R = params.maxEdgeLength * targetSpacing;
    m_grid.build(positions, R);

    // Kandidaten-Dreiecke als Fan um jeden Partikel erzeugen
    struct TriCandidate {
        uint32_t a, b, c;
    };
    std::vector<TriCandidate> candidates;

    // Pro-Partikel-Fan ist unabhaengig von allen anderen Partikeln (nur
    // lesender Zugriff auf positions/normals), deshalb partikelparallel:
    // jeder Thread sammelt in einer eigenen Kandidatenliste + lokalen Stats,
    // die am Ende seriell zusammengefuehrt werden (kein Vector-Race).
    auto processParticle = [&](uint32_t pi, std::vector<TriCandidate>& out, MeshStats& localStats) {
        glm::vec3 p = positions[pi];
        glm::vec3 n = normals[pi];
        TangentBasis basis = computeTangentBasis(n);

        struct Neighbor2D {
            uint32_t id;
            float angle;
        };
        std::vector<Neighbor2D> localNeighbors;
        std::vector<uint32_t> cellIds;
        std::array<int, 3> cell = {
            static_cast<int>(std::floor(p.x / m_grid.cellSize)) - m_grid.origin[0],
            static_cast<int>(std::floor(p.y / m_grid.cellSize)) - m_grid.origin[1],
            static_cast<int>(std::floor(p.z / m_grid.cellSize)) - m_grid.origin[2],
        };
        m_grid.query(cell, positions, R, cellIds);

        for (uint32_t qi : cellIds) {
            if (qi == pi) continue;
            glm::vec3 d = positions[qi] - p;
            if (!acceptEdge(p, positions[qi], n, normals[qi], targetSpacing, sdf, params, localStats)) continue;

            float x = glm::dot(d, basis.u);
            float y = glm::dot(d, basis.v);
            localNeighbors.push_back({qi, atan2f(y, x)});
        }

        if (localNeighbors.size() < 2) return;

        std::sort(localNeighbors.begin(), localNeighbors.end(),
            [](const Neighbor2D& a, const Neighbor2D& b) { return a.angle < b.angle; });

        for (size_t k = 0; k < localNeighbors.size(); ++k) {
            size_t k2 = (k + 1) % localNeighbors.size();
            uint32_t a = localNeighbors[k].id;
            uint32_t b = localNeighbors[k2].id;

            glm::vec3 v0 = positions[a] - p;
            glm::vec3 v1 = positions[b] - p;
            glm::vec3 fn = glm::cross(v0, v1);
            if (glm::length(fn) < 1e-8f) { localStats.degenerate++; continue; }

            // Orientierung vereinheitlichen
            uint32_t i0 = a, i1 = b;
            if (glm::dot(fn, n) < 0.0f) std::swap(i0, i1);

            out.push_back({pi, i0, i1});
        }
    };

    const bool parallel = pool && pool->workerCount() > 0 && N >= 1024;
    if (!parallel) {
        for (uint32_t pi = 0; pi < N; ++pi)
            processParticle(pi, candidates, m_stats);
    } else {
        const unsigned slots = pool->workerCount() + 1; // + Haupt-Thread-Slot
        std::vector<std::vector<TriCandidate>> partCandidates(slots);
        std::vector<MeshStats> partStats(slots);
        pool->parallelFor(N, [&](std::size_t i) {
            unsigned wid = pool->currentWorkerId();
            processParticle(static_cast<uint32_t>(i), partCandidates[wid], partStats[wid]);
        });

        size_t total = 0;
        for (const auto& c : partCandidates) total += c.size();
        candidates.reserve(total);
        for (auto& c : partCandidates)
            candidates.insert(candidates.end(), c.begin(), c.end());
        for (const auto& s : partStats) {
            m_stats.degenerate       += s.degenerate;
            m_stats.wrongOrientation += s.wrongOrientation;
            m_stats.rejectedLength   += s.rejectedLength;
            m_stats.rejectedManifold += s.rejectedManifold;
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
}
