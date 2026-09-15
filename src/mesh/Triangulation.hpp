#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

class ThreadPool;

struct Triangle {
    uint32_t i0, i1, i2;
};

struct Edge {
    uint32_t a, b;
    bool operator==(const Edge& o) const {
        return (a == o.a && b == o.b) || (a == o.b && b == o.a);
    }
};

struct MeshStats {
    int totalTriangles   = 0;
    int degenerate       = 0;
    int wrongOrientation = 0;
    int rejectedLength   = 0;
    int rejectedManifold = 0;
};

struct TangentBasis {
    glm::vec3 u, v;
};

class Triangulation {
public:
    struct Parameters {
        float maxEdgeLength = 1.6f; // als Vielfaches von targetSpacing (Delaunay-Ring)
    };

    // Bei pool == nullptr oder kleiner Partikelzahl bleibt die serielle
    // Referenz-Implementierung aktiv; ab N >= 1024 wird die Kandidaten-
    // Fan-Erzeugung (der O(N^2)-Nachbarsuchen-Anteil) parallelisiert.
    void build(
        const std::vector<glm::vec3>& positions,
        const std::vector<glm::vec3>& normals,
        float targetSpacing,
        const class SDF& sdf,
        const Parameters& params,
        ThreadPool* pool = nullptr
    );

    const std::vector<Triangle>& triangles() const { return m_triangles; }
    const MeshStats& stats() const { return m_stats; }

private:
    TangentBasis computeTangentBasis(glm::vec3 normal) const;
    bool acceptEdge(
        glm::vec3 pa, glm::vec3 pb,
        glm::vec3 na, glm::vec3 nb,
        float targetSpacing, const SDF& sdf, const Parameters& params,
        MeshStats& stats
    ) const;

    std::vector<Triangle> m_triangles;
    MeshStats m_stats;
};
