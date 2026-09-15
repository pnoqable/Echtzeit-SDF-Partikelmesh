#include "SpatialHash.hpp"
#include "../core/ThreadPool.hpp"
#include <cmath>
#include <algorithm>

void SpatialHash::build(const std::vector<glm::vec3>& positions, float cellSize, ThreadPool* pool) {
    m_cells.clear();
    m_pairs.clear();
    m_cellSize = cellSize;
    const size_t N = positions.size();
    if (N == 0) return;

    const bool parallel = pool && pool->workerCount() > 0 && N >= 1024;

    // --- Phase 1: Partikel in ihre Zellen einfuegen ---
    if (!parallel) {
        for (uint32_t i = 0; i < static_cast<uint32_t>(N); ++i) {
            CellKey key = cellOf(positions[i]);
            m_cells[key].push_back(i);
        }
    } else {
        // Jeder Thread schreibt in eine eigene Teil-Map (keine HashMap-Mutation
        // aus mehreren Threads), die am Ende seriell zusammengefuehrt wird.
        const unsigned slots = pool->workerCount() + 1; // + Haupt-Thread-Slot
        std::vector<std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash>> parts(slots);
        pool->parallelFor(N, [&](std::size_t i) {
            parts[pool->currentWorkerId()][cellOf(positions[i])].push_back(static_cast<uint32_t>(i));
        });
        for (auto& part : parts) {
            for (auto& [key, vec] : part) {
                auto it = m_cells.find(key);
                if (it == m_cells.end())
                    m_cells.emplace(key, std::move(vec));
                else
                    it->second.insert(it->second.end(), vec.begin(), vec.end());
            }
        }
    }

    // --- Phase 2: Nachbarpaare (j>i), 27er-Umgebung je Zelle ---
    if (!parallel) {
        for (auto& [cell, ids] : m_cells) {
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        CellKey neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
                        auto it = m_cells.find(neighbor);
                        if (it == m_cells.end()) continue;

                        for (uint32_t i : ids) {
                            for (uint32_t j : it->second) {
                                if (j > i)
                                    m_pairs.push_back({i, j});
                            }
                        }
                    }
                }
            }
        }
        return;
    }

    // Parallel: Zellen read-only durchlaufen, jeder Thread sammelt in einer
    // eigenen Paarliste, die zum Schluss verkettet wird.
    std::vector<CellKey> cells;
    cells.reserve(m_cells.size());
    for (const auto& [key, ids] : m_cells) {
        (void)ids;
        cells.push_back(key);
    }

    const unsigned slots = pool->workerCount() + 1;
    std::vector<std::vector<NeighborPair>> partPairs(slots);
    pool->parallelFor(cells.size(), [&](std::size_t ci) {
        const CellKey cell = cells[ci];
        const std::vector<uint32_t>& ids = m_cells.at(cell);
        std::vector<NeighborPair>& out = partPairs[pool->currentWorkerId()];
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    CellKey neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
                    auto it = m_cells.find(neighbor);
                    if (it == m_cells.end()) continue;

                    for (uint32_t i : ids) {
                        for (uint32_t j : it->second) {
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

SpatialHash::CellKey SpatialHash::cellOf(glm::vec3 position) const {
    return {
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize)),
        static_cast<int>(std::floor(position.z / m_cellSize)),
    };
}

const std::vector<uint32_t>* SpatialHash::idsInCell(CellKey cell) const {
    auto it = m_cells.find(cell);
    return it != m_cells.end() ? &it->second : nullptr;
}

int SpatialHash::particleCountInCell(CellKey cell) const {
    auto it = m_cells.find(cell);
    return it != m_cells.end() ? static_cast<int>(it->second.size()) : 0;
}

std::vector<SpatialHash::CellKey> SpatialHash::occupiedCells() const {
    std::vector<SpatialHash::CellKey> cells;
    cells.reserve(m_cells.size());
    for (const auto& [cell, ids] : m_cells)
        cells.push_back(cell);
    return cells;
}
