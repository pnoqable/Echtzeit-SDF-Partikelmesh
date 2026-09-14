#pragma once
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Minimaler, dependency-freier Thread-Pool fuer die Partikelsimulation.
// macOS libc++ liefert std::execution::par nur seriell, deshalb ein eigener Pool.
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

    // Dinamische Index-Zuteilung: worker holen sich per atomic_fetch_add den
    // naechsten Index. Haupt-Thread arbeitet mit. Nach Ruckkehr sind alle
    // fertig und der Pool kann sofort wiederverwendet werden.
    void parallelFor(std::size_t count,
                     const std::function<void(std::size_t)>& fn);

private:
    void workerLoop();

    std::vector<std::thread>  m_workers;
    std::mutex                m_mutex;
    std::condition_variable   m_cv;
    std::function<void(std::size_t)> m_fn;
    std::atomic<std::size_t>  m_index{0};
    std::size_t               m_count{0};
    unsigned                  m_generation{0};
    std::atomic<unsigned>     m_finished{0};
    bool                      m_destroy{false};
};
