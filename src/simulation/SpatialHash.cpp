#include "SpatialHash.hpp"
#include "../core/ThreadPool.hpp"
#include "../core/Profiler.hpp"
#include <cmath>
#include <algorithm>

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace {

// Zell-Relativ-Indizes (identisch zu `SpatialHash::cellOf(pos) - origin`)
// fuer die Partikel `pos[0..n)`. Skalarer Referenz-Pfad; auf aarch64 wird er
// nur fuer den Rest nach der NEON-Batch-Variante benutzt. Alle Pfade liefern
// bitidentische Ergebnisse, weil `idsInCell(cellOf(p))` die Zellen anspricht.
[[maybe_unused]] void relCellsScalar(const glm::vec3* pos, std::size_t n,
                                     const SpatialHash::CellKey& origin,
                                     float cellSize, SpatialHash::CellKey* out) {
    for (std::size_t i = 0; i < n; ++i) {
        SpatialHash::CellKey c{
            static_cast<int>(std::floor(pos[i].x / cellSize)),
            static_cast<int>(std::floor(pos[i].y / cellSize)),
            static_cast<int>(std::floor(pos[i].z / cellSize)),
        };
        out[i] = { c.x - origin.x, c.y - origin.y, c.z - origin.z };
    }
}

#if defined(__aarch64__)
// NEON: vier Partikel je Lane. Nutzt echte Division (`vdivq_f32`, identisch
// zu skalarer IEEE-Division) + Floor, damit die Keys exakt den skalar
// berechneten entsprechen.
void relCellsNeon(const glm::vec3* pos, std::size_t n,
                  const SpatialHash::CellKey& origin,
                  float cellSize, SpatialHash::CellKey* out) {
    const float32x4_t cs = vdupq_n_f32(cellSize);
    const int32x4_t ox = vdupq_n_s32(origin.x);
    const int32x4_t oy = vdupq_n_s32(origin.y);
    const int32x4_t oz = vdupq_n_s32(origin.z);

    std::size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        float xs[4], ys[4], zs[4];
        for (int k = 0; k < 4; ++k) {
            xs[k] = pos[i + k].x;
            ys[k] = pos[i + k].y;
            zs[k] = pos[i + k].z;
        }
        const int32x4_t ix = vsubq_s32(
            vcvtq_s32_f32(vrndmq_f32(vdivq_f32(vld1q_f32(xs), cs))), ox);
        const int32x4_t iy = vsubq_s32(
            vcvtq_s32_f32(vrndmq_f32(vdivq_f32(vld1q_f32(ys), cs))), oy);
        const int32x4_t iz = vsubq_s32(
            vcvtq_s32_f32(vrndmq_f32(vdivq_f32(vld1q_f32(zs), cs))), oz);
        out[i + 0] = { vgetq_lane_s32(ix, 0), vgetq_lane_s32(iy, 0), vgetq_lane_s32(iz, 0) };
        out[i + 1] = { vgetq_lane_s32(ix, 1), vgetq_lane_s32(iy, 1), vgetq_lane_s32(iz, 1) };
        out[i + 2] = { vgetq_lane_s32(ix, 2), vgetq_lane_s32(iy, 2), vgetq_lane_s32(iz, 2) };
        out[i + 3] = { vgetq_lane_s32(ix, 3), vgetq_lane_s32(iy, 3), vgetq_lane_s32(iz, 3) };
    }
    // Rest skalar (identisches Ergebnis).
    relCellsScalar(pos + i, n - i, origin, cellSize, out + i);
}
#endif

} // namespace

void SpatialHash::build(const std::vector<glm::vec3>& positions, float cellSize, ThreadPool* pool) {
    m_cellIds.clear();
    m_ranges.clear();
    m_cellSize = cellSize;
    const size_t N = positions.size();
    if (N == 0) return;

    const bool parallel = pool && pool->workerCount() > 0 && N >= 1024;

    // Zellraster aus der aktuellen Partikelverteilung ableiten: Das Gitter
    // bleibt absolut (floor(pos / cellSize)), aber nur der Ausschnitt um die
    // Bounding-Box wird als Array vorgehalten. +2 Zellen Padding, damit
    // Rand-Zellen bei den 27er-Nachbarsuchen ohne Overflow auskommen.
    glm::vec3 boxMin = positions[0], boxMax = positions[0];
    for (size_t i = 1; i < N; ++i) {
        boxMin = glm::min(boxMin, positions[i]);
        boxMax = glm::max(boxMax, positions[i]);
    }
    m_origin = cellOf(boxMin);
    const CellKey maxCell = cellOf(boxMax);
    m_dims = { maxCell.x - m_origin.x + 2,
               maxCell.y - m_origin.y + 2,
               maxCell.z - m_origin.z + 2 };
    const std::size_t totalCells =
        static_cast<std::size_t>(m_dims.x) * m_dims.y * m_dims.z;

    // --- Partikel in ihre Zellen einfuegen (kompaktes CSR) ---
    {
        auto _t = prof::Profiler::instance().scoped("grid-phase1");

        std::vector<CellKey> rel(N);
#if defined(__aarch64__)
        relCellsNeon(positions.data(), N, m_origin, cellSize, rel.data());
#else
        relCellsScalar(positions.data(), N, m_origin, cellSize, rel.data());
#endif

        // (ZellIndex, PartikelID)-Eintraege: jeder Partikel beschreibt genau
        // seinen eigenen Slot (race-frei), das Sortieren gruppiert die
        // Partikel je Zelle. So entsteht das kompakte flache Layout, ohne
        // pro-Zelle-Vektoren oder pro-Worker-Teil-Arrays des alten Aufbaus.
        std::vector<std::pair<std::size_t, uint32_t>> entries(N);
        if (parallel) {
            pool->parallelFor(N, [&](std::size_t i) {
                const CellKey r = rel[i];
                entries[i] = { bucketIndex(r.x, r.y, r.z),
                               static_cast<uint32_t>(i) };
            });
        } else {
            for (std::size_t i = 0; i < N; ++i) {
                const CellKey r = rel[i];
                entries[i] = { bucketIndex(r.x, r.y, r.z),
                               static_cast<uint32_t>(i) };
            }
        }
        std::sort(entries.begin(), entries.end());

        m_cellIds.assign(N, 0);
        for (std::size_t i = 0; i < N; ++i)
            m_cellIds[i] = entries[i].second;

        // m_ranges: (start, end) je Zelle, fuer nicht besetzte Zellen leer
        // ({0,0}). Belegte Zellen werden einmal gesammelt (m_occupiedKeys)
        // und fuer Phase 2 sowie occupiedCells() wiederverwendet.
        m_ranges.assign(totalCells, std::pair<uint32_t, uint32_t>{0, 0});
        m_occupiedKeys.clear();
        m_occupiedKeys.reserve(128);
        const std::size_t strideYZ = static_cast<std::size_t>(m_dims.z) * m_dims.y;
        std::size_t begin = 0;
        while (begin < entries.size()) {
            const std::size_t cellIdx = entries[begin].first;
            const int rz = static_cast<int>(cellIdx % static_cast<std::size_t>(m_dims.z));
            const int ry = static_cast<int>((cellIdx / static_cast<std::size_t>(m_dims.z)) % static_cast<std::size_t>(m_dims.y));
            const int rx = static_cast<int>(cellIdx / strideYZ);
            std::size_t end = begin + 1;
            while (end < entries.size() && entries[end].first == cellIdx) ++end;
            m_ranges[cellIdx] = { static_cast<uint32_t>(begin),
                                  static_cast<uint32_t>(end) };
            m_occupiedKeys.push_back({ rx, ry, rz });
            begin = end;
        }
    }
}

SpatialHash::CellKey SpatialHash::cellOf(glm::vec3 position) const {
    return {
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize)),
        static_cast<int>(std::floor(position.z / m_cellSize)),
    };
}

SpatialHash::CellView SpatialHash::rangeOf(CellKey relative) const {
    if (m_ranges.empty()) return {};
    const auto& r = m_ranges[bucketIndex(relative.x, relative.y, relative.z)];
    return { m_cellIds.data() + r.first, r.second - r.first };
}

SpatialHash::CellView SpatialHash::idsInCell(CellKey cell) const {
    const int rx = cell.x - m_origin.x;
    const int ry = cell.y - m_origin.y;
    const int rz = cell.z - m_origin.z;
    if (rx < 0 || rx >= m_dims.x || ry < 0 || ry >= m_dims.y ||
        rz < 0 || rz >= m_dims.z)
        return {};
    return rangeOf({rx, ry, rz});
}

int SpatialHash::particleCountInCell(CellKey cell) const {
    return static_cast<int>(idsInCell(cell).count);
}

std::vector<SpatialHash::CellKey> SpatialHash::occupiedCells() const {
    std::vector<SpatialHash::CellKey> cells;
    cells.reserve(m_occupiedKeys.size());
    for (const CellKey& r : m_occupiedKeys)
        cells.push_back({ r.x + m_origin.x, r.y + m_origin.y, r.z + m_origin.z });
    return cells;
}