#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

// Minimaler, dependency-freier Thread-Pool (von Simulation und Mesh-Erzeugung
// gemeinsam genutzt). macOS libc++ liefert std::execution::par nur seriell,
// deshalb ein eigener Pool.
// Haupt-Thread nimmt an der Arbeit teil; worker() schlummert bis eine neue Aufgabe
// erscheint (Generations-Zaehler + Condition Variable).
class ThreadPool {
public:
    // workerCount = 0  → hardware_concurrency; workerCount = 1 → serial.
    explicit ThreadPool(unsigned workerCount = 0);
    ~ThreadPool();

    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    unsigned workerCount() const { return static_cast<unsigned>(m_workers.size()); }

    // Index des aktuell ausgefuerten Threads (0..workerCount-1 im Worker,
    // == workerCount im Haupt-Thread, sonst maximal-wert). Dient z. B. dazu,
    // pro-Worker-Puffer (Teil-Maps, Teil-Paarlisten) zu adressieren.
    unsigned currentWorkerId() const;

    // Chunk-Groesse der dynamischen Index-Zuteilung: Jeder Teilnehmer holt
    // sich per fetch_add einen kontiguen Bereich von `chunk` Indizes statt
    // Einzel-Indizes. chunk = 1 entspricht dem vorherigen Verhalten
    // (feinkoernigste Zuteilung). Nur zwischen parallelFor-Aufrufen setzen.
    void setChunkSize(std::size_t chunk) { m_chunkSize = chunk > 0 ? chunk : 1; }
    std::size_t chunkSize() const { return m_chunkSize; }

    // Adaptives Chunking: pro parallelFor-Aufruf wird die Blockgroesse auf
    // max(1, count / (Teilnehmer * divisor)) gesetzt (Teilnehmer = Worker +
    // Haupt-Thread). divisor = 0 schaltet die adaptive Berechnung aus und
    // nutzt wieder m_chunkSize. Wert zwischen parallelen Aufrufen aendern.
    void setAdaptiveChunking(unsigned divisor) { m_adaptiveDivisor = divisor; }

    // Dynamische Index-Zuteilung: worker holen sich per atomic_fetch_add den
    // naechsten (Chunk-)Block. Haupt-Thread arbeitet mit. Nach Ruckkehr sind
    // alle fertig und der Pool kann sofort wiederverwendet werden.
    void parallelFor(std::size_t count,
                     const std::function<void(std::size_t)>& fn);

private:
    void workerLoop(unsigned id);
    // Gemeinsame Claim-Schleife (Worker und Haupt-Thread): holt in Schritten
    // von m_chunkSize Indizes und ruft m_fn fuer [begin, end) auf.
    void runChunkedWork();

    std::vector<std::thread>  m_workers;
    std::mutex                m_mutex;
    std::condition_variable   m_cv;
    std::function<void(std::size_t)> m_fn;
    std::atomic<std::size_t>  m_index{0};
    std::size_t               m_chunkSize{1};   // Blockgroesse beim Claimen
    std::size_t               m_count{0};
    unsigned                  m_generation{0};
    std::atomic<unsigned>     m_finished{0};
    unsigned                  m_adaptiveDivisor{8}; // >0: adaptiv statt m_chunkSize (Default)
    bool                      m_destroy{false};
};
