#include "VoronoiDual.hpp"
#include "../core/Profiler.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {

struct EdgeKey {
    uint32_t a, b; // geordnet: a < b
    bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
};
struct EdgeKeyHash {
    size_t operator()(const EdgeKey& k) const {
        return std::hash<uint32_t>()(k.a) ^ (std::hash<uint32_t>()(k.b) << 1);
    }
};

} // namespace

void VoronoiDual::build(
    const std::vector<glm::vec3>& positions,
    const std::vector<glm::vec3>& normals,
    const std::vector<Triangle>& triangles
) {
    auto _t = prof::Profiler::instance().scoped("voronoi");
    m_vertices.clear();
    m_vertexNormals.clear();
    m_cells.clear();
    m_edges.clear();

    const size_t N = positions.size();
    const size_t T = triangles.size();
    if (N == 0 || T == 0) return;

    // 1) Dual-Vertex je Dreieck = Schwerpunkt; je Partikel die inzidenten
    //    Dreiecks-Indizes sammeln. Normale je Dual-Vertex = orientierte
    //    Face-Normale (konsistent aussen, da die Triangulation orientiert ist).
    m_vertices.resize(T);
    m_vertexNormals.resize(T);
    std::vector<std::vector<uint32_t>> perParticle(N);
    for (uint32_t t = 0; t < static_cast<uint32_t>(T); ++t) {
        const Triangle& tri = triangles[t];
        m_vertices[t] = (positions[tri.i0] + positions[tri.i1] + positions[tri.i2]) / 3.0f;
        glm::vec3 fn = glm::cross(positions[tri.i1] - positions[tri.i0],
                                  positions[tri.i2] - positions[tri.i0]);
        float len = glm::length(fn);
        m_vertexNormals[t] = len > 1e-8f ? fn / len : glm::vec3(0.0f);
        perParticle[tri.i0].push_back(t);
        perParticle[tri.i1].push_back(t);
        perParticle[tri.i2].push_back(t);
    }

    // 2) Je Partikel die Zentroide zu einem Ring sortieren: Winkel um die
    //    Partikel-Normale in der Tangentebene (Rechenweg wie in der
    //    Fan-Triangulation, damit die Reihenfolge der Netz-Ring entspricht).
    for (uint32_t pi = 0; pi < static_cast<uint32_t>(N); ++pi) {
        auto& triList = perParticle[pi];
        if (triList.size() < 3) continue; // isolierter Partikel, keine Zelle

        glm::vec3 n = normals[pi];
        glm::vec3 up = std::abs(n.y) < 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
        glm::vec3 u = glm::normalize(glm::cross(up, n));
        glm::vec3 v = glm::cross(n, u);

        std::sort(triList.begin(), triList.end(), [&](uint32_t ta, uint32_t tb) {
            glm::vec3 dA = m_vertices[ta] - positions[pi];
            glm::vec3 dB = m_vertices[tb] - positions[pi];
            float angA = std::atan2(glm::dot(dA, v), glm::dot(dA, u));
            float angB = std::atan2(glm::dot(dB, v), glm::dot(dB, u));
            if (angA == angB) return ta < tb; // stabil bleiben
            return angA < angB;
        });

        Cell cell;
        cell.particle = pi;
        cell.corners.reserve(triList.size());
        for (uint32_t t : triList) cell.corners.push_back(t);
        m_cells.push_back(std::move(cell));
    }

    rebuildEdges();
    rebuildFaces(positions);
}

void VoronoiDual::rebuildFaces(const std::vector<glm::vec3>& positions) {
    m_faceTriangles.clear();
    m_fillVertices.clear();
    if (m_cells.empty() || m_vertices.empty()) return;

    // Jede Zelle als Triangle-Fan um die urspruengliche Partikel-Position:
    // der Partikel (primaler Vertex) wird zusaetzlich in den Vertex-Pool der
    // Zellflaeche aufgenommen und erhaelt dort den zentralen Fan-Radius. So
    // staucht/ueberhoht die Zellflaeche die Krümmung der SDF-Oberflaeche mit,
    // statt jede Zelle als flache Scheibe ihrer Dual-Eckpunkte darzustellen.
    m_fillVertices.reserve(m_vertices.size() + m_cells.size());
    m_fillVertices = m_vertices;
    for (const auto& cell : m_cells) {
        const auto& c = cell.corners;
        if (c.size() < 3) continue;
        m_fillVertices.push_back(positions[cell.particle]);
        const uint32_t center = static_cast<uint32_t>(m_fillVertices.size() - 1);
        const uint32_t n = static_cast<uint32_t>(c.size());
        for (uint32_t k = 0; k < n; ++k) {
            const uint32_t next = (k + 1) % n;
            m_faceTriangles.push_back(Triangle{ center, c[k], c[next] });
        }
    }
}

void VoronoiDual::rebuildEdges() {
    m_edges.clear();
    if (m_cells.empty() || m_vertices.empty()) return;

    // Jede Zellkante eindeutig ablegen. Eine duale Kante (Zentroid von zwei
    // Dreiecken mit gemeinsamer Gitterkante) wird von genau zwei Zellen
    // referenziert -> das geordnete Paar kommt nur einmal in m_edges an.
    std::unordered_set<EdgeKey, EdgeKeyHash> seen;
    m_edges.reserve(m_cells.size());
    for (const auto& cell : m_cells) {
        for (size_t k = 0; k < cell.corners.size(); ++k) {
            uint32_t ca = cell.corners[k];
            uint32_t cb = cell.corners[(k + 1) % cell.corners.size()];
            EdgeKey key{ std::min(ca, cb), std::max(ca, cb) };
            if (seen.insert(key).second)
                m_edges.push_back({ key.a, key.b });
        }
    }
}