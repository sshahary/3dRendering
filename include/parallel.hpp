#pragma once
#include <thread>
#include <vector>
#include <algorithm>
#include <cstddef>

namespace par {
inline unsigned hw_threads(){ unsigned n=std::thread::hardware_concurrency(); return n?n:4u; }

template<class F>
inline void parallel_for(std::size_t begin,std::size_t end,F f,unsigned threads=hw_threads(),std::size_t threshold=4096){
    if(end<=begin) return; std::size_t n=end-begin;
    if(threads<=1 || n<threshold){ for(std::size_t i=begin;i<end;++i) f(i); return; }
    threads = std::min<unsigned>(threads, (unsigned)n);
    std::vector<std::thread> ts; ts.reserve(threads);
    std::size_t chunk=(n+threads-1)/threads;
    for(unsigned t=0;t<threads;++t){
        std::size_t s=begin+t*chunk; if(s>=end) break; std::size_t e=std::min(end,s+chunk);
        ts.emplace_back([=](){ for(std::size_t i=s;i<e;++i) f(i); });
    }
    for(auto& th: ts) th.join();
}
}
