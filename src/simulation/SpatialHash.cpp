#include "SpatialHash.hpp"
#include <cmath>
#include <algorithm>

void SpatialHash::build(const std::vector<glm::vec3>& positions, float cellSize) {
    m_cells.clear();
    m_pairs.clear();
    m_cellSize = cellSize;

    for (uint32_t i = 0; i < static_cast<uint32_t>(positions.size()); ++i) {
        CellKey key = cellOf(positions[i]);
        m_cells[key].push_back(i);
    }

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
}

SpatialHash::CellKey SpatialHash::cellOf(glm::vec3 position) const {
    return {
        static_cast<int>(std::floor(position.x / m_cellSize)),
        static_cast<int>(std::floor(position.y / m_cellSize)),
        static_cast<int>(std::floor(position.z / m_cellSize)),
    };
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
