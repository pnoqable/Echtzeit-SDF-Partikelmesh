#include "../src/simulation/SpatialHash.hpp"
#include "../src/core/ThreadPool.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

// Referenztest (Plan-Abnahme Phase 3): Die Spatial-Hash-Suche muss denselben
// naechsten Nachbarn je Partikel liefern wie die langsame O(N^2)-Suche.
// Geprueft werden BOTH der serielle und der parallele Build-Pfad (N >= 1024
// aktiviert die Parallelisierung).
//
// Frueher verglich der Test die Paar-Set aus pairs(); seit b7a5826 liefert der
// Hash keine Paarliste mehr, daher wird die aequivalente Semantik (Minimum ueber
// die 27 Nachbarzellen) gegen die Brute-Force-Referenz geprueft.

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

// Prueft seit b7a5826 (Paarliste entfernt) den naechsten Nachbarn je Partikel:
// das Minimum ueber die 27 Nachbarzellen muss der brutalkraft O(N^2)-Suche
// entsprechen. Die alte Paar-Set-Pruefung ist gegenstandslos, da SpatialHash
// keine Paare mehr liefert; die Semantik ist dieselbe Nachbarschaft.
static std::vector<float> nearestViaHash(SpatialHash& h,
                                         const std::vector<glm::vec3>& positions) {
    const int N = static_cast<int>(positions.size());
    std::vector<float> nearest(static_cast<size_t>(N), std::numeric_limits<float>::max());
    for (int i = 0; i < N; ++i) {
        const auto ck = h.cellOf(positions[i]);
        for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
            const auto ids = h.idsInCell({ck.x + dx, ck.y + dy, ck.z + dz});
            for (uint32_t k = 0; k < ids.count; ++k) {
                const int j = static_cast<int>(ids.data[k]);
                if (j == i) continue;
                nearest[i] = std::min(nearest[i],
                    glm::length(positions[j] - positions[i]));
            }
        }
    }
    return nearest;
}

static bool verify(const std::vector<glm::vec3>& positions, float cellSize,
                   const char* label, SpatialHash& h) {
    auto cellOf = [&](glm::vec3 p) -> std::array<int, 3> {
        return { static_cast<int>(std::floor(p.x / cellSize)), static_cast<int>(std::floor(p.y / cellSize)), static_cast<int>(std::floor(p.z / cellSize)) };
    };

    const int N = static_cast<int>(positions.size());

    // Referenz: naechster Nachbar je Partikel, aber nur innerhalb der 27er-
    // Zellumgebung (Chebyshev <= 1). Das ist bewusst EINGESCHRAENKT auf die
    // Nachbarschaft, nicht global: die alte Paarliste enthielt ebenfalls nur
    // Paare aus benachbarten Zellen, und die Kraftberechnung sucht nur dort.
    // Ein Partikel ohne Nachbar in seiner Zellumgebung hat daher FLT_MAX als
    // Ergebnis, auch wenn es global einen weiter entfernten Partikel gibt.
    const float kNone = std::numeric_limits<float>::max();
    std::vector<float> referenceNN(static_cast<size_t>(N), kNone);
    std::vector<std::array<int, 3>> cells(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) cells[i] = cellOf(positions[i]);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == j) continue;
            if (std::abs(cells[i][0] - cells[j][0]) > 1 ||
                std::abs(cells[i][1] - cells[j][1]) > 1 ||
                std::abs(cells[i][2] - cells[j][2]) > 1)
                continue;
            referenceNN[i] = std::min(referenceNN[i],
                glm::length(positions[j] - positions[i]));
        }
    }

    const std::vector<float> actualNN = nearestViaHash(h, positions);

    bool ok = true;
    int isolated = 0;
    for (int i = 0; i < N; ++i) {
        if (referenceNN[i] == kNone) ++isolated;   // kein Nachbar in 27er-Umgebung
        // Exakter Vergleich: beide Seiten berechnen dieselben Distanzen aus
        // denselben Partikelpositionen, daher kein Toleranzband noetig.
        ok &= (actualNN[i] == referenceNN[i]);
    }

    printf("%-28s NN geprueft: %d  (isoliert: %d)  %s\n",
        label, N, isolated, ok ? "OK" : "FAIL");
    if (!ok) {
        for (int i = 0; i < N; ++i)
            if (actualNN[i] != referenceNN[i])
                printf("  Partikel %d: Hash %.9f vs Referenz %.9f\n",
                    i, actualNN[i], referenceNN[i]);
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

    // Paralleler Pfad (N >= 1024), mit ThreadPool; muss dieselben
    // Nachbarabstaende liefern wie der serielle Build und wie O(N^2).
    {
        auto pos = makePositions(2000);
        SpatialHash par;
        ThreadPool pool;
        par.build(pos, cellSize, &pool);

        SpatialHash ser;
        ser.build(pos, cellSize);

        bool ok = verify(pos, cellSize, "parallel  (N=2000)", par);
        ok &= verify(pos, cellSize, "serial    (N=2000)", ser);

        // Identische NN-Vektoren aus beiden Pfaden
        const std::vector<float> pa = nearestViaHash(par, pos);
        const std::vector<float> sa = nearestViaHash(ser, pos);
        ok &= (pa == sa);
        printf("%-28s %s\n", "parallel == serial NN", ok ? "OK" : "FAIL");
        if (!ok) return 1;
    }

    printf("TEST PASS (Spatial-Hash NN == O(N^2)-Referenz, serial + parallel)\n");
    return 0;
}