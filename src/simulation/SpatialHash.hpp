#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <unordered_map>
#include <cstdint>

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
    struct CellKeyHash {
        size_t operator()(const CellKey& k) const {
            size_t h = std::hash<int>()(k.x);
            h = h * 31 + std::hash<int>()(k.y);
            h = h * 31 + std::hash<int>()(k.z);
            return h;
        }
    };

    void build(const std::vector<glm::vec3>& positions, float cellSize);
    const std::vector<NeighborPair>& pairs() const { return m_pairs; }
    CellKey cellOf(glm::vec3 position) const;
    int particleCountInCell(CellKey cell) const;
    std::vector<CellKey> occupiedCells() const;
    float cellSize() const { return m_cellSize; }

private:
    std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> m_cells;
    std::vector<NeighborPair> m_pairs;
    float m_cellSize = 1.0f;
};