#include "../src/simulation/SpatialHash.hpp"
#include "../src/core/ThreadPool.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

// Referenztest (Plan-Abnahme Phase 3): Die Spatial-Hash-Suche muss dieselben
// Nachbarpaare liefern wie die langsame O(N^2)-Vergleichssuche ueber alle
// Zellen-Nachbarn (27 Umgebung). Geprueft werden BOTH der serielle und der
// parallele Build-Pfad (N >= 1024 aktiviert die Parallelisierung).

static std::vector<glm::vec3> makePositions(int N) {
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
    return positions;
}

static bool verify(const std::vector<glm::vec3>& positions, float cellSize,
                   const char* label, SpatialHash& h) {
    auto cellOf = [&](glm::vec3 p) -> std::array<int, 3> {
        return { static_cast<int>(std::floor(p.x / cellSize)), static_cast<int>(std::floor(p.y / cellSize)), static_cast<int>(std::floor(p.z / cellSize)) };
    };

    const int N = static_cast<int>(positions.size());
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

    printf("%-28s Paare: %-6zu Referenz: %-6zu %s\n",
        label, h.pairs().size(), reference.size(), ok ? "OK" : "FAIL");
    if (!ok) {
        printf("  aktuelle Paare, nicht in Referenz: ");
        for (auto& p : actual) if (!reference.count(p)) printf("(%d,%d) ", p.first, p.second);
        printf("\n  Referenz-Paare, nicht aktuell: ");
        for (auto& p : reference) if (!actual.count(p)) printf("(%d,%d) ", p.first, p.second);
        printf("\n");
    }
    return ok;
}

int main() {
    const float cellSize = 0.15f;

    // Serieller Pfad (N < Parallelschwelle 1024)
    {
        auto pos = makePositions(200);
        SpatialHash h;
        h.build(pos, cellSize);
        if (!verify(pos, cellSize, "serial (N=200)", h)) return 1;
    }

    // Paralleler Pfad (N >= 1024), mit ThreadPool; muessen dieselben Paare
    // liefern wie die O(N^2)-Referenz und wie der serielle Build.
    {
        auto pos = makePositions(2000);
        SpatialHash par;
        ThreadPool pool;
        par.build(pos, cellSize, &pool);

        SpatialHash ser;
        ser.build(pos, cellSize);

        bool ok = verify(pos, cellSize, "parallel  (N=2000)", par);
        ok &= verify(pos, cellSize, "serial    (N=2000)", ser);

        // Identisches Paar-Set aus beiden Pfaden
        std::set<std::pair<int,int>> pa, sa;
        for (const auto& p : par.pairs()) pa.emplace((int)p.i, (int)p.j);
        for (const auto& p : ser.pairs()) sa.emplace((int)p.i, (int)p.j);
        ok &= pa == sa;
        printf("%-28s %s\n", "parallel == serial set", ok ? "OK" : "FAIL");
        if (!ok) return 1;
    }

    printf("TEST PASS (Spatial-Hash == O(N^2)-Referenz, serial + parallel)\n");
    return 0;
}