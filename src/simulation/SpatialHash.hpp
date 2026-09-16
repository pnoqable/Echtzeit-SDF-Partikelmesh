#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

class ThreadPool;

// Spatial-Hash ueber ein an die aktuelle Partikelverteilung angepasstes,
// linearisiertes Zellen-Array (statt `unordered_map`):
//
//   - Das Absolut-Gitter bleibt `floor(position / cellSize)`, Zellen werden
//     nur relativ zur Box-Ecke `m_origin` indiziert. `cellOf()` und die
//     Zell-Keys der Renderer-APIs sind dadurch unveraendert.
//   - Zugriffe (`idsInCell`, Nachbarzellen) sind O(1)-Array-Zugriffe ohne
//     Hash-Lookups; das Zusammenfuehren der pro-Worker-Teile ist eine
//     flache Vektor-Merge-Schleife statt Map-Merging.
//   - Die Indizierung in Phase 1 ist per SIMD (NEON) auf vier Partikel je
//     Lane vektorisiert; auf x86 faellt sie auf den skalar identischen Pfad
//     zurueck.
class SpatialHash {
public:
    struct NeighborPair {
        uint32_t i, j;
    };

    struct CellKey {
        int x, y, z;
        bool operator==(const CellKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    // Bei pool == nullptr oder kleiner Partikelzahl bleibt die serielle
    // Referenz-Implementierung aktiv; ab N >= 1024 wird parallelisiert
    // (Einfuegen in pro-Worker-Teil-Arrays + Paargenerierung ueber Zellen).
    void build(const std::vector<glm::vec3>& positions, float cellSize, ThreadPool* pool = nullptr);
    void clear() { m_buckets.clear(); m_pairs.clear(); }
    const std::vector<NeighborPair>& pairs() const { return m_pairs; }
    CellKey cellOf(glm::vec3 position) const;

    // Zugriff auf die Partikel-Indizes einer Zelle (nullptr falls leer oder
    // ausserhalb des Zellrasters). Wird von der partikelparallelen
    // Kraftschleife in ParticleSystem::relax() genutzt, damit jeder Particle
    // die 27 Nachbarzellen seines Partikels traversieren kann, ohne die
    // globale Paarliste zu durchlaufen.
    const std::vector<uint32_t>* idsInCell(CellKey cell) const;
    int particleCountInCell(CellKey cell) const;
    std::vector<CellKey> occupiedCells() const;
    float cellSize() const { return m_cellSize; }

private:
    // linearisierter Zellindex (relative Koordinaten sind im Raster [0, dims)).
    std::size_t bucketIndex(int rx, int ry, int rz) const {
        return (static_cast<std::size_t>(rx) * m_dims.y + ry) * m_dims.z + rz;
    }

    std::vector<std::vector<uint32_t>> m_buckets;
    CellKey m_origin{ 0, 0, 0 }; // Absolut-Zelle der Box-Ecke (ceil-seite)
    CellKey m_dims{ 0, 0, 0 };   // Raster-Groesse (mit 1 Zelle Padding)
    std::vector<NeighborPair> m_pairs;
    float m_cellSize = 1.0f;
};