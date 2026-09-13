#include "../src/mesh/Triangulation.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include "../src/debug/Metrics.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

// M5: Persistente Topologie im Langzeittest.
// Szene: Kugel, 1000 Partikel, App-Defaults, Seed 42.
// Ablauf: 60 s relaxieren, EINMAL triangulieren, danach Indexbuffer nie mehr
// aendern. Ueber weitere 5 min Laufzeit (T=10 s, 60 s, 5 min) wird geprueft:
//   1) der Indexbuffer (system.triangles) bleibt bit-identisch;
//   2) nur Vertexpositionen (particles[i].position) aktualisieren sich;
//   3) die Mesh-Qualitaet und die Abstands-/SDF-Metriken bleiben in Toleranz.
int main() {
    const int N = 1000;
    const float dt = 1.0f / 60.0f;
    SphereSDF sphere({ 0, 0, 0 }, 1.0f);
    float h = std::sqrt(4.0f * glm::pi<float>() / static_cast<float>(N));

    ParticleSystem sys;
    sys.parameters = SimulationParameters();
    sys.initialize(N, sphere.boundsMin(), sphere.boundsMax(), 42);
    sys.projectToSDF(sphere);
    for (int f = 0; f < 3600; ++f) sys.relax(dt, sphere); // 60 s Vorkonvergenz

    std::vector<glm::vec3> pos, nrm;
    for (auto& p : sys.particles) { pos.push_back(p.position); nrm.push_back(p.normal); }

    Triangulation::Parameters params;
    params.maxEdgeLength = 1.6f;
    Triangulation tri;
    tri.build(pos, nrm, h, sphere, params);
    sys.triangles = tri.triangles();
    const std::vector<Triangle> indexSnapshot = sys.triangles;
    const MeshStats triStats = tri.stats();
    int expectedF = 2 * N - 4;

    sys.buildSpatialHash();
    debug::SimulationMetrics initial = debug::evaluate(sys, sphere, h);

    printf("Indexbuffer initial: F=%d (erwartet %d)  degen=%d orient=%d\n",
        static_cast<int>(indexSnapshot.size()), expectedF, triStats.degenerate, triStats.wrongOrientation);
    printf("  minWinkel %.1f°  maxAspect %.2f  poor %d  avgPhi %.2e  avgDist %.4f\n",
        initial.mesh.minAngleDeg, initial.mesh.maxAspectRatio, initial.mesh.poorTriangles,
        initial.sdf.avgAbsPhi, initial.distribution.avgDist);
    printf("  maxVertexDrift(h): 0.000\n");

    // Metrik-Log-Phasen: +10 s, +60 s, +5 min (kumulativ).
    const int phases[] = { 600, 3600, 18000 };
    const char* phaseNames[] = { "+10 s", "+60 s", "+5 min" };
    bool ok = true;
    int phase = 0;
    int lastFrame = 0;
    for (int targetFrame : phases) {
        for (int f = lastFrame + 1; f <= targetFrame; ++f) sys.relax(dt, sphere);

        // 1) Indexbuffer unverändert?
        bool indexSame = sys.triangles.size() == indexSnapshot.size();
        if (indexSame)
            for (size_t i = 0; i < indexSnapshot.size(); ++i)
                if (sys.triangles[i].i0 != indexSnapshot[i].i0 ||
                    sys.triangles[i].i1 != indexSnapshot[i].i1 ||
                    sys.triangles[i].i2 != indexSnapshot[i].i2) { indexSame = false; break; }

        // 2) Nur Vertexpositionen bewegen sich: maximale Verschiebung in Einheiten h.
        float maxDrift = 0.0f;
        for (int i = 0; i < N; ++i)
            maxDrift = std::max(maxDrift, glm::length(sys.particles[i].position - pos[i]));

        // 3) Qualität + Abstands-/SDF-Metriken.
        sys.buildSpatialHash();
        debug::SimulationMetrics m = debug::evaluate(sys, sphere, h);

        bool phaseOk = indexSame && maxDrift > 0.0f && maxDrift < 0.3f * h
            && m.mesh.poorTriangles == 0
            && m.mesh.minAngleDeg > initial.mesh.minAngleDeg * 0.8f
            && m.mesh.maxAspectRatio < initial.mesh.maxAspectRatio * 1.2f + 0.5f
            && m.sdf.avgAbsPhi < 1e-3f
            && m.distribution.avgDist > 0.9f * h && m.distribution.avgDist < 1.1f * h;
        ok = ok && phaseOk;

        printf("%-7s F=%zu Index-buffer %s  Drift %.3f h  minWinkel %.1f°  maxAspect %.2f  poor %d  avgPhi %.2e  avgDist %.4f  %s\n",
            phaseNames[phase], sys.triangles.size(), indexSame ? "identisch" : "GEAENDERT",
            maxDrift / h, m.mesh.minAngleDeg, m.mesh.maxAspectRatio, m.mesh.poorTriangles,
            m.sdf.avgAbsPhi, m.distribution.avgDist, phaseOk ? "OK" : "FAIL");

        lastFrame = targetFrame;
        ++phase;
    }

    printf("\n%s\n", ok ? "TEST PASS (persistente Topologie stabil, nur Vertices aktualisiert)"
                        : "TEST FAIL");
    return ok ? 0 : 1;
}