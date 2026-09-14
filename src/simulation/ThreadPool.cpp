#include "ThreadPool.hpp"

ThreadPool::ThreadPool(unsigned workerCount) {
    unsigned hw = std::thread::hardware_concurrency();
    if (workerCount == 0)
        workerCount = hw > 0 ? hw : 4;
    m_workers.reserve(workerCount > 0 ? workerCount - 1 : 0);
    for (unsigned i = 1; i < workerCount; ++i)
        m_workers.emplace_back(&ThreadPool::workerLoop, this);
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_destroy = true;
        ++m_generation;   // aufwecken, falls schlummernd
    }
    m_cv.notify_all();
    for (auto& t : m_workers) t.join();
}

// Worker-Thread: laeuft permanent und schlaeft zwischen Aufgaben.
// m_generation aendert sich bei jeder neuen Aufgabe; m_finished zaehlt
// fertiggestellte Worker (reset per parallelFor).
void ThreadPool::workerLoop() {
    unsigned activeGen = 0;
    std::unique_lock<std::mutex> lk(m_mutex);

    for (;;) {
        m_cv.wait(lk, [&]{ return m_destroy || m_generation != activeGen; });
        if (m_destroy) return;
        activeGen = m_generation;
        lk.unlock();

        for (;;) {
            std::size_t i = m_index.fetch_add(1, std::memory_order_relaxed);
            if (i >= m_count) break;
            m_fn(i);
        }

        lk.lock();
        if (m_finished.fetch_add(1, std::memory_order_relaxed) + 1 == m_workers.size())
            m_cv.notify_all();
    }
}

// Haupt-Thread signalisiert neue Aufgabe, arbeitet ebenfalls mit und wartet
// dann bis alle Worker fertig sind (kein Spin, sondern CV-Wait).
void ThreadPool::parallelFor(std::size_t count,
                             const std::function<void(std::size_t)>& fn) {
    if (count == 0) return;

    // Serialer Pfad: keine Worker oder Aufgabe zu klein
    if (m_workers.empty() || count < 128) {
        for (std::size_t i = 0; i < count; ++i) fn(i);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(m_mutex);
        m_fn    = fn;
        m_count = count;
        m_index.store(0, std::memory_order_relaxed);
        m_finished.store(0, std::memory_order_relaxed);
        ++m_generation;
    }
    m_cv.notify_all();

    for (;;) {
        std::size_t i = m_index.fetch_add(1, std::memory_order_relaxed);
        if (i >= count) break;
        fn(i);
    }

    std::unique_lock<std::mutex> lk(m_mutex);
    m_cv.wait(lk, [&]{
        return m_finished.load(std::memory_order_relaxed)
               == static_cast<unsigned>(m_workers.size());
    });
}
