// parallel.hpp
#pragma once
#include <thread>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace par {

inline unsigned hw_threads() {
    unsigned t = std::thread::hardware_concurrency();
    return t ? t : 4;
}

template <class Fn>
inline void parallel_for(std::size_t begin, std::size_t end, Fn fn, unsigned threads = 0) {
    const std::size_t N = (end > begin) ? (end - begin) : 0;
    if (N == 0) return;

    unsigned T = threads ? threads : hw_threads();
    if (T <= 1 || N < 1024) {
        for (std::size_t i = begin; i < end; ++i) fn(i);
        return;
    }

    T = std::min<unsigned>(T, (unsigned)N);
    std::vector<std::thread> pool; pool.reserve(T);
    std::size_t chunk = (N + T - 1) / T;

    for (unsigned t = 0; t < T; ++t) {
        std::size_t s = begin + t * chunk;
        std::size_t e = std::min(begin + (t + 1) * chunk, end);
        if (s >= e) break;
        pool.emplace_back([=]() {
            for (std::size_t i = s; i < e; ++i) fn(i);
        });
    }
    for (auto& th : pool) th.join();
}

}
