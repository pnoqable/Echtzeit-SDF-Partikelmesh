#pragma once

// Konvergenz-Erkennung fuer die Initialrelaxation (Umsetzungsplan §7.3).
// Nur fuer die Tests gedacht: Statt einer festen Frame-Anzahl wird so lange
// relaxiert, bis die Verteilung stabil ist, gedeckelt auf eine Obergrenze.
// Die App selbst relaxiert weiterhin durchgehend (kein Button-Gating).
//
// Kriterien (ueber `stableFrames` aufeinanderfolgende Frames):
//   - mittlere Geschwindigkeit |v| unter `speedTolerance * h / dt`;
//   - Standardabweichung der NN-Abstaende unter `spacingTolerance * h`;
//   - maximaler SDF-Fehler |phi| unter `phiTolerance * h`.

#include "Metrics.hpp"
#include "../simulation/ParticleSystem.hpp"
#include "../simulation/SDF.hpp"
#include <cmath>

namespace debug {

struct ConvergenceOptions {
    int   maxFrames     = 3600; // Obergrenze (Schutz vor Endlosschleifen)
    int   stableFrames  = 60;   // Hysterese: Kriterium muss konstant halten
    float speedTolerance  = 0.002f; // avg|v| < speedTolerance * h / dt
    float spacingTolerance = 0.05f; // stdDev(d) < spacingTolerance * h
    float phiTolerance    = 0.1f;  // max|phi| < phiTolerance * h
};

struct ConvergenceReport {
    int   framesUsed = 0;
    bool  converged  = false;
};

inline ConvergenceReport relaxUntilConverged(
    ParticleSystem& system, const SDF& sdf, float dt,
    const ConvergenceOptions& opt = ConvergenceOptions())
{
    ConvergenceReport rep;
    int stable = 0;
    for (int f = 1; f <= opt.maxFrames; ++f) {
        system.relax(dt, sdf);
        const float h = std::sqrt(sdf.surfaceArea()
            / static_cast<float>(system.particles.size()));
        system.buildSpatialHash();
        const SimulationMetrics m = evaluate(system, sdf, h);
        const bool ok =
            m.avgSpeed          < opt.speedTolerance   * (h / dt) &&
            m.distribution.stdDev < opt.spacingTolerance * h &&
            m.sdf.maxAbsPhi       < opt.phiTolerance    * h;
        if (ok) {
            if (++stable >= opt.stableFrames) {
                rep.framesUsed = f;
                rep.converged = true;
                break;
            }
        } else {
            stable = 0;
        }
        rep.framesUsed = f;
    }
    return rep;
}

}