#include "../src/mesh/VoronoiDual.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include "../src/debug/Convergence.hpp"
#include <glm/glm.hpp>
#include <cstdio>
#include <unordered_map>
#include <vector>

// Zentroid-Dual (diskrete Voronoi-Tesselation) der Kugel-Triangulation.
// Das Dual muss eine geschlossene, orientierbare Flaeche bilden:
//   - jede Zelle >= 3 Ecken;
//   - jede duale Kante wird von genau 2 Zellen geteilt (manifold + closed,
//     jede Gitterkante aus genau 2 Dreiecken, deren Zentroide in beiden
//     Nachbarzellen benachbart sind);
//   - Euler-Charakteristik der Kugel: V - E + F = 2.
int main() {
    int N = 1500;
    SphereSDF sdf(glm::vec3(0.0f), 1.0f);
    ParticleSystem sys;
    sys.parameters = SimulationParameters();
    sys.initialize(N, sdf.boundsMin(), sdf.boundsMax(), 42);
    sys.projectToSDF(sdf);
    const float dt = 1.0f / 60.0f;
    debug::ConvergenceOptions conv;
    conv.maxFrames = 1800;
    debug::relaxUntilConverged(sys, sdf, dt, conv);

    std::vector<glm::vec3> pos, nrm;
    for (const auto& p : sys.particles) { pos.push_back(p.position); nrm.push_back(p.normal); }

    Triangulation::Parameters params;
    params.maxEdgeLength = 1.6f;
    Triangulation tri;
    float h = std::sqrt(sdf.surfaceArea() / static_cast<float>(N));
    tri.build(pos, nrm, h, sdf, params);

    VoronoiDual dual;
    dual.build(pos, nrm, tri.triangles());

    bool ok = true;

    size_t zelleMin3 = 0;
    for (const auto& c : dual.cells())
        if (c.corners.size() < 3) zelleMin3++;
    printf("Zellen: %zu  (mit <3 Ecken: %zu)\n", dual.cells().size(), zelleMin3);
    if (zelleMin3) ok = false;

    auto K = [](unsigned a, unsigned b) -> unsigned long long {
        if (a > b) std::swap(a, b);
        return (static_cast<unsigned long long>(a) << 32) | b;
    };
    std::unordered_map<unsigned long long, int> edgeCount;
    for (const auto& c : dual.cells())
        for (size_t k = 0; k < c.corners.size(); ++k)
            edgeCount[K(c.corners[k], c.corners[(k + 1) % c.corners.size()])]++;

    int badEdges = 0;
    for (const auto& [k, cnt] : edgeCount)
        if (cnt != 2) badEdges++;
    printf("Kanten: %zu  (dedup edges(): %zu, davon !=2 Zellen: %d)\n",
        edgeCount.size(), dual.edges().size(), badEdges);
    if (badEdges || dual.edges().size() != edgeCount.size()) ok = false;

    size_t V = tri.triangles().size(); // duale Vertices = Dreiecke
    size_t E = dual.edges().size();
    size_t F = dual.cells().size();
    long long euler = static_cast<long long>(V) - static_cast<long long>(E) + static_cast<long long>(F);
    printf("Euler: V=%zu E=%zu F=%zu  -> V-E+F=%lld (erwartet 2)\n", V, E, F, euler);
    if (euler != 2) ok = false;

    printf(ok ? "TEST PASS\n" : "TEST FAIL\n");
    return ok ? 0 : 1;
}