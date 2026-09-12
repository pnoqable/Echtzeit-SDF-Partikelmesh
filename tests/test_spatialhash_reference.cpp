#include "../src/simulation/SpatialHash.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

// Referenztest (Plan-Abnahme Phase 3): Die Spatial-Hash-Suche muss dieselben
// Nachbarpaare liefern wie die langsame O(N^2)-Vergleichssuche ueber alle
// Zellen-Nachbarn (27 Umgebung).
int main() {
    const float cellSize = 0.15f;
    const int N = 200;
    std::vector<glm::vec3> positions;
    positions.reserve(N);
    // deterministisch, dicht genug fuer Ueberlappungen
    unsigned x = 12345;
    for (int i = 0; i < N; ++i) {
        x = x * 1664525u + 1013904223u;
        float u = (x % 1000000u) / 1000000.0f;
        x = x * 1664525u + 1013904223u;
        float v = (x % 1000000u) / 1000000.0f;
        x = x * 1664525u + 1013904223u;
        float w = (x % 1000000u) / 1000000.0f;
        positions.emplace_back(1.5f * u, 1.5f * v, 1.5f * w);
    }
    positions[0] = {0.0f, 0.0f, 0.0f};
    positions[1] = {0.1f, 0.0f, 0.0f};
    positions[2] = {0.16f, 0.0f, 0.0f};

    SpatialHash h;
    h.build(positions, cellSize);

    auto cellOf = [&](glm::vec3 p) -> std::array<int, 3> {
        return { static_cast<int>(std::floor(p.x / cellSize)), static_cast<int>(std::floor(p.y / cellSize)), static_cast<int>(std::floor(p.z / cellSize)) };
    };
    auto cellsEqual = [](const std::array<int,3>& a, const std::array<int,3>& b) {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    };

    // Referenz: Paare (i,j), j>i, deren Zellen Chebyshev-Distanz <= 1 haben
    std::set<std::pair<int,int>> reference;
    for (int i = 0; i < N; ++i) {
        auto ci = cellOf(positions[i]);
        for (int j = i + 1; j < N; ++j) {
            auto cj = cellOf(positions[j]);
            if (std::abs(ci[0]-cj[0]) <= 1 && std::abs(ci[1]-cj[1]) <= 1 && std::abs(ci[2]-cj[2]) <= 1)
                reference.emplace(i, j);
        }
    }

    std::set<std::pair<int,int>> actual;
    for (const auto& p : h.pairs())
        actual.emplace(static_cast<int>(p.i), static_cast<int>(p.j));

    bool ok = reference == actual;
    // Deduplizierung: jede Kante genau einmal
    ok &= actual.size() == h.pairs().size();

    printf("Paare: %zu  Referenz: %zu\n", h.pairs().size(), reference.size());
    if (!ok) {
        printf("  aktuelle Paare, nicht in Referenz: ");
        for (auto& p : actual) if (!reference.count(p)) printf("(%d,%d) ", p.first, p.second);
        printf("\n  Referenz-Paare, nicht aktuell: ");
        for (auto& p : reference) if (!actual.count(p)) printf("(%d,%d) ", p.first, p.second);
        printf("\nTEST FAIL\n");
        return 1;
    }
    printf("TEST PASS (Spatial-Hash == O(N^2)-Referenz)\n");
    return 0;
}