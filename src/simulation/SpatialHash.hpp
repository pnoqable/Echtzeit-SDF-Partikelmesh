#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <utility>

class ThreadPool;

// Spatial-Hash ueber ein an die aktuelle Partikelverteilung angepasstes,
// linearisiertes Zellen-Array (statt `unordered_map`), in kompakter CSR-Form:
//
//   - Das Absolut-Gitter bleibt `floor(position / cellSize)`, Zellen werden
//     nur relativ zur Box-Ecke `m_origin` indiziert. `cellOf()` und die
//     Zell-Keys der Renderer-APIs sind dadurch unveraendert.
//   - Partikel liegen flach in `m_cellIds`, nach Zelle gruppiert; `m_ranges`
//     haelt pro linearisierter Zelle das Intervall [start, end) in diesem
//     Array. Es gibt keine `std::vector`-Objekte mehr je Zelle: weder Heap-
//     Allokationen pro belegter Zelle noch leere-Zelle-Objekte.
//   - Zugriffe (`idsInCell`, Nachbarzellen) bleiben O(1)-Array-Zugriffe ohne
//     Hash-Lookups; die einmal gesammelten belegten Zellen (`m_occupiedKeys`)
//     speisen Phase 2 (Paare) und `occupiedCells()` ohne Raster-Scans.
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

    // Kompakter Sicht auf die Partikel einer Zelle: `data[0..count)` zeigt in
    // `m_cellIds`; count == 0 bedeutet leer bzw. ausserhalb des Rasters.
    struct CellView {
        const uint32_t* data = nullptr;
        uint32_t count = 0;
    };

    // Bei pool == nullptr oder kleiner Partikelzahl bleibt die serielle
    // Referenz-Implementierung aktiv; ab N >= 1024 wird parallelisiert
    // (race-freie Eintraege je Partikel-Slot + Paargenerierung ueber die
    // belegten Zellen).
    void build(const std::vector<glm::vec3>& positions, float cellSize, ThreadPool* pool = nullptr);
    void clear() {
        m_cellIds.clear();
        m_ranges.clear();
        m_occupiedKeys.clear();
    }
    CellKey cellOf(glm::vec3 position) const;

    // Zugriff auf die Partikel-Indizes einer Zelle (leer falls die Zelle leer
    // oder ausserhalb des Zellrasters liegt). Wird von der partikelparallelen
    // Kraftschleife in ParticleSystem::relax() genutzt, damit jeder Particle
    // die 27 Nachbarzellen seines Partikels traversieren kann, ohne die
    // globale Paarliste zu durchlaufen.
    CellView idsInCell(CellKey cell) const;
    int particleCountInCell(CellKey cell) const;
    std::vector<CellKey> occupiedCells() const;
    float cellSize() const { return m_cellSize; }

private:
    // linearisierter Zellindex (relative Koordinaten sind im Raster [0, dims)).
    std::size_t bucketIndex(int rx, int ry, int rz) const {
        return (static_cast<std::size_t>(rx) * m_dims.y + ry) * m_dims.z + rz;
    }

    // Sicht auf die Partikel einer RELATIV indizierten Zelle (kein Origin-
    // Abzug, keine Bounds-Prüfung durch den Aufrufer) fuer die internen
    // Schleifen. Leer wenn die Zelle ausserhalb des Rasters liegt.
    CellView rangeOf(CellKey relative) const;

    std::vector<uint32_t> m_cellIds;                     // flach: alle Partikel, nach Zelle gruppiert
    std::vector<std::pair<uint32_t, uint32_t>> m_ranges; // (start, end) je linearisierter Zelle
    std::vector<CellKey> m_occupiedKeys;                 // relative Zellkeys belegter Zellen (Aufbau-Reihenfolge)
    CellKey m_origin{ 0, 0, 0 }; // Absolut-Zelle der Box-Ecke (ceil-seite)
    CellKey m_dims{ 0, 0, 0 };   // Raster-Groesse (mit 1 Zelle Padding)
    float m_cellSize = 1.0f;
};