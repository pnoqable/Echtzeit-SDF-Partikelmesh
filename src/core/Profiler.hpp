#pragma once

// Leichtgewichtiger Stichproben-Profiler fuer die Per-Frame-Pipeline
// (Grid, Kraefte, Integration, Projektion). Grundlage fuer die
// GPU-Entscheidung (Umsetzungsplan §13): Erst CPU-Profil, dann portieren,
// wenn ~grid/~forces dominieren. Ausserdem liefern die Stage-Zeiten den
// "getrennt gemessenen" Vorher/Nachher-Vergleich beim GPU-Port.
//
// Compile-Zeit aktivieren: -DSDFPROFILING=1 (CMake-Option SDFPROFILING).
// Deaktiviert sind alle Aufrufe No-ops und werden wegoptimiert.
//
// Messung nur vom Haupt-Thread: Die Scopes umspannen parallelFor-Aufrufe,
// gemessen wird also die Wandzeit (die Groesse, die fuer das Frame-Budget
// zaehlt). Kein Locking, keine Atomics noetig.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string_view>
#include <unordered_map>
#include <vector>

#if defined(SDFPROFILING) && SDFPROFILING != 0
#define SDFPROFILE_ENABLED 1
#else
#define SDFPROFILE_ENABLED 0
#endif

namespace prof {

inline constexpr bool enabled = SDFPROFILE_ENABLED != 0;

struct StageCounter {
    std::uint64_t count  = 0;   // Anzahl Scopes seit reset()
    std::uint64_t totalNs = 0;  // kumulierte Zeit seit reset()
    std::uint64_t lastNs  = 0;  // letzter Einzelaufruf
    std::uint64_t frameNs = 0;  // Summe der Aufrufe im aktuellen Frame
    std::uint64_t minNs   = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t maxNs   = 0;
};

class Profiler {
public:
    static constexpr bool enabled = SDFPROFILE_ENABLED != 0;

    struct Scoped {
        Profiler*     p  = nullptr;
        StageCounter* c  = nullptr;
        std::chrono::steady_clock::time_point t0{};

        Scoped() = default;
        explicit Scoped(Profiler& prof, const char* name) {
            if constexpr (enabled) {
                p  = &prof;
                c  = &prof.counter(name);
                t0 = std::chrono::steady_clock::now();
            }
        }
        Scoped(Scoped&& o) noexcept : p(o.p), c(o.c), t0(o.t0) { o.p = nullptr; }
        Scoped& operator=(Scoped&& o) noexcept {
            if (this != &o) { p = o.p; c = o.c; t0 = o.t0; o.p = nullptr; }
            return *this;
        }
        ~Scoped() {
            if constexpr (enabled) {
                if (!p) return; // moved-from
                const std::uint64_t ns =
                    (std::uint64_t)(std::chrono::steady_clock::now() - t0).count();
                ++c->count;
                c->totalNs += ns;
                c->frameNs += ns;
                c->lastNs   = ns;
                if (ns < c->minNs) c->minNs = ns;
                if (ns > c->maxNs) c->maxNs = ns;
            }
        }
    };

    StageCounter& counter(const char* name) {
        auto it = m_stages.find(name);
        if (it == m_stages.end())
            it = m_stages.emplace(name, StageCounter{}).first;
        return it->second;
    }

    // RAII-Objekt: misst bis zum Ende des umgebenden Blocks.
    Scoped scoped(const char* name) { return Scoped(*this, name); }

    // Am Ende jedes Frames aufrufen: Per-Frame-Akku zuruecksetzen, damit die
    // UI "letzter Frame" anzeigen kann (kumulierte Werte bleiben erhalten).
    void endFrame() {
        if constexpr (!enabled) return;
        for (auto& [name, v] : m_stages) { (void)name; v.frameNs = 0; }
    }

    // Zaehler und Akkumulatoren zuruecksetzen (Bench-Warmup, Messperiode).
    void reset() {
        if constexpr (!enabled) return;
        m_stages.clear();
    }

    struct StageStat {
        const char*   name;
        StageCounter  c;
    };

    // Momentbild aller registrierten Stages, lexikographisch nach Name
    // sortiert (stabil fuer Tabellen und Vergleiche).
    std::vector<StageStat> stages() const {
        std::vector<StageStat> out;
        if constexpr (!enabled) return out;
        out.reserve(m_stages.size());
        for (const auto& [name, v] : m_stages)
            out.push_back({name, v});
        std::sort(out.begin(), out.end(), [](const StageStat& a, const StageStat& b) {
            return std::string_view(a.name) < std::string_view(b.name);
        });
        return out;
    }

    static Profiler& instance() { static Profiler p; return p; }

private:
    Profiler() = default;
    std::unordered_map<const char*, StageCounter> m_stages;
};

} // namespace prof

#undef SDFPROFILE_ENABLED