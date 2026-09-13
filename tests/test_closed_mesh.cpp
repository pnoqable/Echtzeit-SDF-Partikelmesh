#include "../src/mesh/Triangulation.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>

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

// App-Defaults (SimulationParameters), 60 s echte Relaxation, dann Triangulation.
// Erwartung: 0 Randkanten und F = 2V-4 fuer jeden Seed.
int main() {
    const int N = 1000;
    SphereSDF sphere({ 0, 0, 0 }, 1.0f);
    float sp = std::sqrt(4.0f * glm::pi<float>() / static_cast<float>(N));
    const float dt = 1.0f / 60.0f;
    const int seeds[] = { 42, 7, 1234, 555, 3, 11, 77, 2001, 31415, 9999 };
    bool ok = true;

    for (int seed : seeds) {
        ParticleSystem sys;
        sys.parameters = SimulationParameters();
        sys.initialize(N, sphere.boundsMin(), sphere.boundsMax(), seed);
        sys.projectToSDF(sphere);
        for (int f = 0; f < 3600; ++f) sys.relax(dt, sphere);

        std::vector<glm::vec3> pos, nrm;
        for (auto& p : sys.particles) { pos.push_back(p.position); nrm.push_back(p.normal); }

        Triangulation::Parameters params;
        params.maxEdgeLength = 1.6f;
        Triangulation tri;
        tri.build(pos, nrm, sp, sphere, params);

        auto st = tri.stats();
        int b = boundary(tri.triangles());
        int expectF = 2 * N - 4;
        bool seedOk = (st.totalTriangles == expectF) && b == 0
            && st.degenerate == 0 && st.wrongOrientation == 0;
        ok = ok && seedOk;
        printf("seed=%5d F=%4d erwartet=%4d Rand=%2d degen=%d orient=%d %s\n",
               seed, st.totalTriangles, expectF, b, st.degenerate, st.wrongOrientation,
               seedOk ? "OK" : "FAIL");
    }

    printf("\n%s\n", ok ? "TEST PASS (10/10 geschlossen)" : "TEST FAIL");
    return ok ? 0 : 1;
}