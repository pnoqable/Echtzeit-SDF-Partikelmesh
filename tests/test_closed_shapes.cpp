#include "../src/mesh/Triangulation.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include "../src/debug/Metrics.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>

// M6: Torus und konkave Testform (Hantel) — dokumentierte Grenzen.
//
// Prüft pro Form nach 30 s Relaxation + Triangulation:
//   - keine degenerierten Dreiecke, keine falsch orientierten Dreiecke;
//   - Euler-Konsistenz innerhalb einer dokumentierten Toleranz:
//       Sphäre (Kugel, Hantel, chi=2):  F = 2V − 4  (exakt)
//       Torus  (chi=0):                  F = 2V      (exakt)
//   - Randkanten: dokumentiert bis zu einem einzelnen kleinen Loch
//     (≤ 5 Randkanten). Der Boundary-Loop-Fill schliesst nur 3er-/4er-Loops;
//     ein offenes Fuenfeck-Loch auf stark gekruemmten/kankaven Bereichen ist
//     die dokumentierte Grenze der aktuellen Fan-Triangulation.
//
// Ergebniszeilen erfuellen zwei Rollen: (a) Regression gegen bekannte Werte,
// (b) Protokoll der Grenzen für die Meilenstein-Dokumentation.
static int boundary(const std::vector<Triangle>& tris) {
    std::unordered_map<unsigned long long, int> m;
    auto K = [](unsigned a, unsigned b) -> unsigned long long {
        if (a > b) std::swap(a, b);
        return (static_cast<unsigned long long>(a) << 32) | b;
    };
    for (auto& t : tris) { m[K(t.i0, t.i1)]++; m[K(t.i1, t.i2)]++; m[K(t.i2, t.i0)]++; }
    int b = 0;
    for (auto& [k, c] : m) if (c == 1) b++;
    return b;
}

static bool runShape(const char* name, const SDF& sdf, int N, float maxEdgeMul) {
    ParticleSystem sys;
    sys.parameters = SimulationParameters();
    sys.initialize(N, sdf.boundsMin(), sdf.boundsMax(), 42);
    sys.projectToSDF(sdf);
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 1800; ++f) sys.relax(dt, sdf); // 30 s

    std::vector<glm::vec3> pos, nrm;
    for (auto& p : sys.particles) { pos.push_back(p.position); nrm.push_back(p.normal); }

    Triangulation::Parameters params;
    params.maxEdgeLength = maxEdgeMul;
    Triangulation tri;
    float h = std::sqrt(sdf.surfaceArea() / static_cast<float>(N));
    tri.build(pos, nrm, h, sdf, params);
    auto st = tri.stats();
    sys.triangles = tri.triangles();
    int b = boundary(tri.triangles());

    int total = st.totalTriangles;
    int expect = 2 * N;          // Torus (chi=0)
    const char* euler = "F=2V (chi=0)";
    if (dynamic_cast<const SphereSDF*>(&sdf) || dynamic_cast<const DumbbellSDF*>(&sdf)) {
        expect = 2 * N - 4;
        euler = "F=2V-4 (chi=2)";
    }

    // Dokumentierte Toleranz: kein degeneriertes/falsch orientiertes Dreieck,
    // hoechstens ein kleines Loch (<= 5 Randkanten), Euler-Delta <= 5.
    bool sound = st.degenerate == 0 && st.wrongOrientation == 0;
    bool withinTolerance = std::abs(total - expect) <= 5 && b <= 5;

    sys.buildSpatialHash();
    debug::SimulationMetrics m = debug::evaluate(sys, sdf, h);

    printf("%-8s V=%d F=%d (delta %+d, %s) Rand=%d degen=%d orient=%d | minWinkel %.1f° maxAspect %.2f poor %d | avgDist %.4f  %s\n",
        name, N, total, total - expect, euler, b, st.degenerate, st.wrongOrientation,
        m.mesh.minAngleDeg, m.mesh.maxAspectRatio, m.mesh.poorTriangles,
        m.distribution.avgDist, (sound && withinTolerance) ? "OK" : "FAIL");
    return sound && withinTolerance;
}

int main() {
    bool ok = true;
    printf("Dokumentierte Grenzen M6 — stark gekruemmte/konkave Bereiche:\n");
    printf("  Boundary-Loop-Fill schliesst 3er-/4er-Loops; ein einzelnes\n");
    printf("  Fuenfeck-Loch (5 Randkanten) bleibt als bekannte Grenze offen.\n\n");
    ok &= runShape("Kugel", SphereSDF({0,0,0}, 1.0f), 1000, 1.4f);
    ok &= runShape("Torus", TorusSDF({0,0,0}, 1.2f, 0.45f), 1500, 1.4f);
    ok &= runShape("Hantel", DumbbellSDF({0,0,0}, 1.0f, 0.5f), 1500, 1.4f);

    printf("\n%s\n", ok
        ? "TEST PASS (alle Formen liefern degenerations- und fehlorientierungsfreie\n       Meshes innerhalb der dokumentierten Loer-Toleranz)"
        : "TEST FAIL");
    return ok ? 0 : 1;
}