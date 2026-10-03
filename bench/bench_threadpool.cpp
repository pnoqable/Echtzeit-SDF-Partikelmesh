// Benchmark: ThreadPool -> Einzel-Index-Zuteilung (chunk=1) vs. Chunk-Allokation.
//
// Misst mehrere representative Workloads auf einem ThreadPool:
//   1. fein:  out[i] = i            -> dominiert vom Atomik-Overhead je Element
//   2. kraft: vec3-akkumulation     -> mittelschwere, kraft-aehnliche Schleife
//   3. grid:  SpatialHash::build()  -> echter App-Workload (CSR-Build, 20k Pts)
//
// Vergleich je chunk-Groesse; Warmup + mehrere Durchlaeufe, Minimum wird
// berichtet. Ergebnis: relativer Speedup vs. chunk = 1 (alter Zustand).

#include "../src/core/ThreadPool.hpp"
#include "../src/simulation/SpatialHash.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cinttypes>
#include <cstdio>
#include <random>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct BenchResult {
    double msLight   = 0.0;
    double msForce   = 0.0;
    double msGrid008 = 0.0;
    double msGrid005 = 0.0;
};

// N synthetische Partikel auf einer Kugeloberflaeche (r = 1), wie im App-Fall.
std::vector<glm::vec3> makeSpherePoints(std::size_t n) {
    std::vector<glm::vec3> out(n);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    for (auto& p : out) {
        const float theta = 2.0f * 3.14159265358979f * u(rng);
        const float phi   = std::acos(1.0f - 2.0f * u(rng));
        p = { std::sin(phi) * std::cos(theta),
              std::sin(phi) * std::sin(theta),
              std::cos(phi) };
    }
    return out;
}

template <typename Fn>
double measureMin(ThreadPool& pool, std::size_t chunk,
                  std::size_t warmup, std::size_t iters, Fn&& fn) {
    pool.setChunkSize(chunk);
    for (std::size_t w = 0; w < warmup; ++w) fn();
    double best = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k < iters; ++k) {
        const auto t0 = Clock::now();
        fn();
        const auto t1 = Clock::now();
        const double ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        best = std::min(best, ms);
    }
    return best;
}

void report(const char* name, double ref, double cur) {
    std::printf("  %-9s %9.3f ms  (speedup %6.2fx vs chunk=1)\n",
                name, cur, ref > 0.0 ? ref / cur : 0.0);
}

} // namespace

int main() {
    const std::size_t N  = 20000;
    const std::size_t warmup = 3;
    const std::size_t iters  = 7;

    // Pool mit voller Hardware-Parallelitaet (wie die App: workerCount = 0).
    ThreadPool pool(0);
    const unsigned workers = pool.workerCount();
    std::printf("workers = %u, N = %zu\n\n", workers, N);

    // Feste-Chunk-Sweep und Baseline bewusst im NICHT-adaptiven Modus messen
    // (der Pool-Default ist seit 72173d1 adaptiv mit divisor=8).
    pool.setAdaptiveChunking(0);

    std::vector<uint32_t> out(N, 0u);
    const auto positions = makeSpherePoints(N);
    SpatialHash grid;
    // Batch je chunk-Groesse, damit der Grid-Build pro Durchlauf neu aufgebaut
    // wird (die `build`-Schleife nutzt den Pool via parallelFor).
    const std::vector<std::size_t> chunks = { 1, 2, 4, 8, 16, 32, 64, 128,
                                              256, 512, 1024 };

    // Referenz: chunk = 1 (dynamische Zuteilung wie zuvor).
    double refLight = 0.0, refForce = 0.0, refGrid008 = 0.0, refGrid005 = 0.0;
    {
        pool.setChunkSize(1);
        refLight = measureMin(pool, 1, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) { out[i] = static_cast<uint32_t>(i); });
        });
        refForce = measureMin(pool, 1, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) {
                // Kraft-aehnlicher Kern: liest 8 "Nachbarn", akkumuliert vec3.
                glm::vec3 acc{0.0f};
                const glm::vec3& p = positions[i];
                for (int k = 1; k <= 8; ++k) {
                    const glm::vec3& q = positions[(i * 7 + static_cast<std::size_t>(k) * 3) % N];
                    const glm::vec3 d = q - p;
                    const float d2 = glm::dot(d, d);
                    acc += d / (d2 + 1e-6f);
                }
                out[i] = static_cast<uint32_t>(acc.x + acc.y + acc.z);
            });
        });
        refGrid008 = measureMin(pool, 1, warmup, iters, [&] {
            grid.build(positions, 0.08f, &pool);
        });
        refGrid005 = measureMin(pool, 1, warmup, iters, [&] {
            grid.build(positions, 0.05f, &pool);
        });
    }

    std::printf("Baseline chunk=1:  light %.3f ms | force %.3f ms | grid0.08 %.3f ms | grid0.05 %.3f ms\n\n",
                refLight, refForce, refGrid008, refGrid005);

    for (const std::size_t c : chunks) {
        std::printf("chunk = %4zu:\n", c);
        report("light", refLight, measureMin(pool, c, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) { out[i] = static_cast<uint32_t>(i); });
        }));
        report("force", refForce, measureMin(pool, c, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) {
                glm::vec3 acc{0.0f};
                const glm::vec3& p = positions[i];
                for (int k = 1; k <= 8; ++k) {
                    const glm::vec3& q = positions[(i * 7 + static_cast<std::size_t>(k) * 3) % N];
                    const glm::vec3 d = q - p;
                    const float d2 = glm::dot(d, d);
                    acc += d / (d2 + 1e-6f);
                }
                out[i] = static_cast<uint32_t>(acc.x + acc.y + acc.z);
            });
        }));
        report("grid0.08", refGrid008, measureMin(pool, c, warmup, iters, [&] {
            grid.build(positions, 0.08f, &pool);
        }));
        report("grid0.05", refGrid005, measureMin(pool, c, warmup, iters, [&] {
            grid.build(positions, 0.05f, &pool);
        }));
        std::printf("\n");
    }

    // Adaptiver Modus: chunk = max(1, count / (Teilnehmer * divisor)).
    // Teilnehmer = workerCount + 1 (Haupt-Thread arbeitet mit).
    std::printf("=== adaptiv (chunk = count / (teilnehmer * divisor)) ===\n");
    pool.setChunkSize(1);
    for (const unsigned d : { 4u, 8u, 16u }) {
        std::printf("divisor = %2u:\n", d);
        const std::size_t effChunk = std::max<std::size_t>(
            1, N / ((std::size_t)workers + 1) / d);
        std::printf("  (effektiver chunk bei N=20000: %zu)\n", effChunk);

        pool.setAdaptiveChunking(d);
        report("light", refLight, measureMin(pool, 1, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) { out[i] = static_cast<uint32_t>(i); });
        }));
        report("force", refForce, measureMin(pool, 1, warmup, iters, [&] {
            pool.parallelFor(N, [&](std::size_t i) {
                glm::vec3 acc{0.0f};
                const glm::vec3& p = positions[i];
                for (int k = 1; k <= 8; ++k) {
                    const glm::vec3& q = positions[(i * 7 + static_cast<std::size_t>(k) * 3) % N];
                    const glm::vec3 d = q - p;
                    const float d2 = glm::dot(d, d);
                    acc += d / (d2 + 1e-6f);
                }
                out[i] = static_cast<uint32_t>(acc.x + acc.y + acc.z);
            });
        }));
        report("grid0.08", refGrid008, measureMin(pool, 1, warmup, iters, [&] {
            grid.build(positions, 0.08f, &pool);
        }));
        report("grid0.05", refGrid005, measureMin(pool, 1, warmup, iters, [&] {
            grid.build(positions, 0.05f, &pool);
        }));
        std::printf("\n");
    }
    pool.setAdaptiveChunking(0);

    return 0;
}