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
    m_buckets.clear();
    m_pairs.clear();
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

    m_buckets.assign(static_cast<std::size_t>(m_dims.x) * m_dims.y * m_dims.z,
                     std::vector<uint32_t>{});

    // --- Phase 1: Partikel in ihre Zellen einfuegen ---
    {
        auto _t = prof::Profiler::instance().scoped("grid-phase1");

        std::vector<CellKey> rel(N);
#if defined(__aarch64__)
        relCellsNeon(positions.data(), N, m_origin, cellSize, rel.data());
#else
        relCellsScalar(positions.data(), N, m_origin, cellSize, rel.data());
#endif

        if (!parallel) {
            for (uint32_t i = 0; i < static_cast<uint32_t>(N); ++i) {
                const CellKey r = rel[i];
                m_buckets[bucketIndex(r.x, r.y, r.z)].push_back(i);
            }
        } else {
            // Jeder Thread schreibt in eine eigene Teil-Zell-Array-Struktur
            // (keine Mutation eines geteilten Arrays), die am Ende seriell
            // Zelle fuer Zelle zusammengefuehrt wird.
            const unsigned slots = pool->workerCount() + 1; // + Haupt-Thread-Slot
            std::vector<std::vector<std::vector<uint32_t>>> parts(slots);
            for (auto& part : parts)
                part.assign(m_buckets.size(), std::vector<uint32_t>{});

            pool->parallelFor(N, [&](std::size_t i) {
                const CellKey r = rel[i];
                parts[pool->currentWorkerId()][bucketIndex(r.x, r.y, r.z)]
                    .push_back(static_cast<uint32_t>(i));
            });

            for (std::size_t idx = 0; idx < m_buckets.size(); ++idx) {
                for (const auto& part : parts) {
                    const auto& v = part[idx];
                    if (!v.empty())
                        m_buckets[idx].insert(m_buckets[idx].end(), v.begin(), v.end());
                }
            }
        }
    }

    // --- Phase 2: Nachbarpaare (j>i), 27er-Umgebung je Zelle ---
    {
        auto _t = prof::Profiler::instance().scoped("grid-phase2");

        if (!parallel) {
            for (int rx = 0; rx < m_dims.x; ++rx)
                for (int ry = 0; ry < m_dims.y; ++ry)
                    for (int rz = 0; rz < m_dims.z; ++rz) {
                        const auto& ids = m_buckets[bucketIndex(rx, ry, rz)];
                        if (ids.empty()) continue;
                        for (int dx = -1; dx <= 1; ++dx) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dz = -1; dz <= 1; ++dz) {
                                    const int nx = rx + dx, ny = ry + dy, nz = rz + dz;
                                    if (nx < 0 || nx >= m_dims.x || ny < 0 ||
                                        ny >= m_dims.y || nz < 0 || nz >= m_dims.z)
                                        continue;
                                    const auto& nids = m_buckets[bucketIndex(nx, ny, nz)];
                                    if (nids.empty()) continue;
                                    for (uint32_t i : ids)
                                        for (uint32_t j : nids)
                                            if (j > i)
                                                m_pairs.push_back({i, j});
                                }
                            }
                        }
                    }
            return;
        }

        // Parallel: belegte Zellen read-only durchlaufen, jeder Thread sammelt
        // in einer eigenen Paarliste, die zum Schluss verkettet wird.
        std::vector<CellKey> cells;
        cells.reserve(m_buckets.size());
        for (int rx = 0; rx < m_dims.x; ++rx)
            for (int ry = 0; ry < m_dims.y; ++ry)
                for (int rz = 0; rz < m_dims.z; ++rz) {
                    const auto& ids = m_buckets[bucketIndex(rx, ry, rz)];
                    if (!ids.empty())
                        cells.push_back({rx, ry, rz});
                }

        const unsigned slots = pool->workerCount() + 1;
        std::vector<std::vector<NeighborPair>> partPairs(slots);
        pool->parallelFor(cells.size(), [&](std::size_t ci) {
            const CellKey cell = cells[ci];
            const std::vector<uint32_t>& ids =
                m_buckets[bucketIndex(cell.x, cell.y, cell.z)];
            std::vector<NeighborPair>& out = partPairs[pool->currentWorkerId()];
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const int nx = cell.x + dx, ny = cell.y + dy, nz = cell.z + dz;
                        if (nx < 0 || nx >= m_dims.x || ny < 0 ||
                            ny >= m_dims.y || nz < 0 || nz >= m_dims.z)
                            continue;
                        const auto& nids = m_buckets[bucketIndex(nx, ny, nz)];
                        if (nids.empty()) continue;

                        for (uint32_t i : ids) {
                            for (uint32_t j : nids) {
                                if (j > i)
                                    out.push_back({i, j});
                            }
                        }
                    }
                }
            }
        });

        size_t total = 0;
        for (const auto& p : partPairs) total += p.size();
        m_pairs.reserve(total);
        for (auto& p : partPairs)
            m_pairs.insert(m_pairs.end(), p.begin(), p.end());
    }
}

SpatialHash::CellKey SpatialHash::cellOf(glm::vec3 position) const {
    return {
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize)),
        static_cast<int>(std::floor(position.z / m_cellSize)),
    };
}

const std::vector<uint32_t>* SpatialHash::idsInCell(CellKey cell) const {
    if (m_buckets.empty()) return nullptr;
    const int rx = cell.x - m_origin.x;
    const int ry = cell.y - m_origin.y;
    const int rz = cell.z - m_origin.z;
    if (rx < 0 || rx >= m_dims.x || ry < 0 || ry >= m_dims.y ||
        rz < 0 || rz >= m_dims.z)
        return nullptr;
    return &m_buckets[bucketIndex(rx, ry, rz)];
}

int SpatialHash::particleCountInCell(CellKey cell) const {
    const auto* ids = idsInCell(cell);
    return ids ? static_cast<int>(ids->size()) : 0;
}

std::vector<SpatialHash::CellKey> SpatialHash::occupiedCells() const {
    std::vector<SpatialHash::CellKey> cells;
    for (int rx = 0; rx < m_dims.x; ++rx)
        for (int ry = 0; ry < m_dims.y; ++ry)
            for (int rz = 0; rz < m_dims.z; ++rz) {
                if (m_buckets[bucketIndex(rx, ry, rz)].empty()) continue;
                cells.push_back({ rx + m_origin.x, ry + m_origin.y, rz + m_origin.z });
            }
    return cells;
}