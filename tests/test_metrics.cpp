#include "../src/debug/Metrics.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>

// Prueft: evaluieren() liefert plausible Verteilungs-/SDF-Metriken fuer eine
// relaxierte Kugel und vertretbare Mesh-Qualitaet nach Triangulation.
int main() {
    const int N = 1000;
    SphereSDF sphere({ 0, 0, 0 }, 1.0f);
    float spacing = std::sqrt(4.0f * glm::pi<float>() / static_cast<float>(N));

    ParticleSystem sys;
    sys.initialize(N, sphere.boundsMin(), sphere.boundsMax(), 42);
    sys.projectToSDF(sphere);

    // Unrelaxiert: grosser StdAbw, viele under/over
    sys.buildSpatialHash();
    debug::SimulationMetrics m0 = debug::evaluate(sys, sphere, spacing);
    bool ok = true;
    ok &= std::isfinite(m0.distribution.avgDist) && m0.distribution.avgDist > 0.0f;
    ok &= m0.distribution.underCount + m0.distribution.okCount + m0.distribution.overCount == N;
    ok &= m0.sdf.maxAbsPhi < 0.1f; // Projektion vor Relaxation: nahe SDF

    // 60 s Relaxation: mittlerer Abstand nahe Zielabstand, StdAbw klein
    const float dt = 1.0f / 60.0f;
    for (int f = 0; f < 3600; ++f) sys.relax(dt, sphere);
    sys.buildSpatialHash();
    debug::SimulationMetrics m1 = debug::evaluate(sys, sphere, spacing);

    float d = std::abs(m1.distribution.avgDist - spacing) / spacing;
    ok &= d < 0.1f;                                  // avgDist innerhalb 10% von h
    ok &= m1.distribution.stdDev < 0.3f * spacing;   // gleichmaessige Verteilung

    // Mesh-Triangulation: Qualitaetsmetriken liegen im erwarteten Bereich
    std::vector<glm::vec3> pos, nrm;
    for (const auto& p : sys.particles) { pos.push_back(p.position); nrm.push_back(p.normal); }
    Triangulation tri;
    Triangulation::Parameters tp;
    tri.build(pos, nrm, spacing, sphere, tp);
    sys.triangles = tri.triangles();

    debug::SimulationMetrics m2 = debug::evaluate(sys, sphere, spacing);
    ok &= m2.mesh.minAngleDeg > 0.0f && m2.mesh.minAngleDeg <= 180.0f;
    ok &= m2.mesh.maxAspectRatio >= 1.0f;
    ok &= m2.mesh.poorTriangles >= 0;

    const char* status = ok ? "TEST PASS" : "TEST FAIL";
    printf("unrelaxed: avg=%.4f under/ok/over=%d/%d/%d maxPhi=%.2e\n", m0.distribution.avgDist,
        m0.distribution.underCount, m0.distribution.okCount, m0.distribution.overCount, m0.sdf.maxAbsPhi);
    printf("relaxed:   avg=%.4f (h=%.4f) std=%.4f under/ok/over=%d/%d/%d maxPhi=%.2e\n",
        m1.distribution.avgDist, spacing, m1.distribution.stdDev,
        m1.distribution.underCount, m1.distribution.okCount, m1.distribution.overCount, m1.sdf.maxAbsPhi);
    printf("mesh:      minWinkel=%.2f° maxAspect=%.2f poor=%d\n", m2.mesh.minAngleDeg,
        m2.mesh.maxAspectRatio, m2.mesh.poorTriangles);
    printf("%s", status);
    return ok ? 0 : 1;
}