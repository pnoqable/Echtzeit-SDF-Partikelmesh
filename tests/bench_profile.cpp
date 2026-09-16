// Headless-Performanzmessung der Sim-Pipeline (GPU-Entscheidung, §13).
//
// Misst je Partikelzahl N die Stage-Anteile eines Relax-Frames:
//   grid      – Spatial-Hash-Aufbau (buildSpatialHash, je Substep)
//   forces    – Kraft-Akkumulation (paarweise Abstossung, je Substep)
//   integrate – Positionsintegration + Tangentialprojektion (je Substep)
//   project   – SDF-Projektion (je Substep)
//   sim       – Wandzeit des gesamten relax()-Aufrufs
//   evaluate  – buildSpatialHash + debug::evaluate (einmalig, "Debug-UI-Zyklus")
//   triangulate – Mesh-Triangulation (einmalig, nach dem Loop)
//
// Kompilieren mit Profiling:
//   g++ -std=c++20 -O2 -DSDFPROFILING=1 -I build/debug/_deps/glm-src -I src \
//       src/core/ThreadPool.cpp src/simulation/PrimitiveSDF.cpp \
//       src/simulation/SpatialHash.cpp src/simulation/ParticleSystem.cpp \
//       src/mesh/Triangulation.cpp src/debug/Metrics.cpp \
//       tests/bench_profile.cpp -o bench_profile -lm -pthread
//
// Aufruf: bench_profile [N [WARMUP [MESS]]]
//   N     = Partikelzahl (0 oder kein Argument: Sweep 1000..10000)
//   WARMUP= verworfenen Warmup-Frames (Standard 200)
//   MESS  = gemessene Frames (Standard 1000)

#include "../src/core/Profiler.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include "../src/mesh/Triangulation.hpp"
#include "../src/debug/Metrics.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <vector>

namespace {

const char* kNames[5] = { "grid", "forces", "integrate", "project", "sim" };

// Bilder je Stage (Summe des letzten Frames) in die Akkumulatoren schreiben.
void scrape(std::vector<std::vector<double>>& acc,
            const std::vector<prof::Profiler::StageStat>& snap) {
    for (int k = 0; k < 5; ++k) {
        double v = 0.0;
        for (const auto& s : snap)
            if (std::string_view(s.name) == kNames[k]) {
                v = static_cast<double>(s.c.frameNs) * 1e-6;
                break;
            }
        acc[k].push_back(v);
    }
}

double meanOf(const std::vector<double>& v) {
    double s = 0.0;
    for (double x : v) s += x;
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}

double quantile(const std::vector<double>& v, double q) {
    if (v.empty()) return 0.0;
    std::vector<double> c = v;
    std::sort(c.begin(), c.end());
    return c[static_cast<std::size_t>(q * static_cast<double>(c.size() - 1))];
}

void runSingle(int N, int warmup, int frames) {
    SphereSDF sdf({ 0.0f, 0.0f, 0.0f }, 1.0f);
    ParticleSystem sys;
    sys.parameters = SimulationParameters();
    sys.initialize(N, sdf.boundsMin(), sdf.boundsMax(), 42);
    sys.projectToSDF(sdf);
    const float sp =
        std::sqrt(4.0f * glm::pi<float>() / static_cast<float>(N));
    const float dt = 1.0f / 60.0f;

    for (int f = 0; f < warmup; ++f)
        sys.relax(dt, sdf);
    prof::Profiler::instance().reset();

    std::vector<std::vector<double>> acc(5);
    for (int f = 0; f < frames; ++f) {
        {
            auto t = prof::Profiler::instance().scoped("sim");
            (void)t;
            sys.relax(dt, sdf);
        }
        scrape(acc, prof::Profiler::instance().stages());
        prof::Profiler::instance().endFrame();
    }

    // Einmalige Kosten nach der Messperiode (verschmutzen die Samples nicht).
    {
        auto t = prof::Profiler::instance().scoped("evaluate");
        (void)t;
        sys.buildSpatialHash();
        debug::evaluate(sys, sdf, sp);
    }
    {
        auto t = prof::Profiler::instance().scoped("triangulate");
        (void)t;
        std::vector<glm::vec3> pos, nrm;
        pos.reserve(N); nrm.reserve(N);
        for (const auto& p : sys.particles) {
            pos.push_back(p.position);
            nrm.push_back(p.normal);
        }
        Triangulation::Parameters triParams;
        triParams.maxEdgeLength = 1.6f;
        Triangulation tri;
        tri.build(pos, nrm, sp, sdf, triParams, &sys.pool());
    }

    std::printf("N=%d substeps=%d warmup=%d frames=%d\n",
        N, sys.parameters.substeps, warmup, frames);
    std::printf("  %-10s %10s %10s %10s %9s\n",
        "stage", "mean_ms", "p95_ms", "min_ms", "sim-%");
    const double simMean = meanOf(acc[4]);
    for (int k = 0; k < 5; ++k) {
        const double m = meanOf(acc[k]);
        std::printf("  %-10s %10.3f %10.3f %10.3f %8.1f%%\n",
            kNames[k], m, quantile(acc[k], 0.95),
            *std::min_element(acc[k].begin(), acc[k].end()),
            simMean > 0.0 ? 100.0 * m / simMean : 0.0);
    }
    const auto once = prof::Profiler::instance().stages();
    for (const auto& s : once) {
        if (std::string_view(s.name) == "evaluate" ||
            std::string_view(s.name) == "triangulate")
            std::printf("  %-10s %10.3f (einmalig)\n", s.name,
                static_cast<double>(s.c.totalNs) * 1e-6);
    }
    std::printf("\n");

    // CSV (eine Zeile je Stage, zum Vergleichen ueber N hinweg).
    for (int k = 0; k < 5; ++k) {
        const double m = meanOf(acc[k]);
        std::printf("CSV bench %d %s %.4f %.4f %.2f\n", N, kNames[k], m,
            quantile(acc[k], 0.95),
            simMean > 0.0 ? 100.0 * m / simMean : 0.0);
    }
}

} // namespace

int main(int argc, char** argv) {
    const int sweepN[] = { 1000, 2500, 5000, 10000 };
    if (argc >= 2 && std::atoi(argv[1]) > 0) {
        const int warmup = argc >= 3 ? std::atoi(argv[2]) : 200;
        const int frames = argc >= 4 ? std::atoi(argv[3]) : 1000;
        runSingle(std::atoi(argv[1]), warmup, frames);
        return 0;
    }
    for (int N : sweepN)
        runSingle(N, 200, 1000);
    return 0;
}