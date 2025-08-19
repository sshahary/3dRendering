#include "mesh_loader.hpp"
#include <fstream>
#include <sstream>
#include <string_view>
#include <algorithm>
#include <cctype>

namespace {
inline std::string_view ltrim(std::string_view s){ size_t i=0; while(i<s.size() && std::isspace((unsigned char)s[i])) ++i; return s.substr(i); }
inline std::string_view rtrim(std::string_view s){ size_t i=s.size(); while(i>0 && std::isspace((unsigned char)s[i-1])) --i; return s.substr(0,i); }
inline std::string_view trim (std::string_view s){ return rtrim(ltrim(s)); }

inline bool parseInt(std::string_view s, int& v){
    s = trim(s); if(s.empty()) return false;
    int sign=1; size_t i=0; if(s[0]=='-'){sign=-1;i=1;} else if(s[0]=='+'){i=1;} if(i==s.size()) return false;
    long long acc=0; for(; i<s.size(); ++i){ unsigned char c=s[i]; if(c<'0'||c>'9') return false; acc=acc*10+(c-'0'); if(acc>0x7fffffff) return false; }
    v = (int)acc*sign; return true;
}
inline bool resolveIndex(int raw, size_t vcount, int& out){
    if(vcount==0) return false;
    if(raw>0) out = raw-1;
    else if(raw<0){ long long idx=(long long)vcount+raw; if(idx<0) return false; out=(int)idx; }
    else return false;
    return out>=0 && (size_t)out<vcount;
}
}

bool MeshLoader::loadOBJ(const std::string& path){
    positions.clear(); faces.clear(); edges.clear();
    std::ifstream is(path); if(!is) return false;
    positions.reserve(1024); faces.reserve(1024);
    std::string line;
    while(std::getline(is,line)){
        if(line.empty()) continue;
        std::string_view sv = trim(line);
        if(sv.empty() || sv[0]=='#') continue;

        std::string s_line{sv};
        std::istringstream ss{s_line};

        std::string tag; ss >> tag;
        if(tag=="v"){
            float x=0,y=0,z=0; if(ss>>x>>y>>z) positions.push_back({x,y,z});
        } else if(tag=="f"){
            PolyFace f; std::string tok;
            while(ss>>tok){
                std::string_view tv(tok);
                if(auto s = tv.find('/'); s!=std::string_view::npos) tv = tv.substr(0,s);
                int raw=0; if(!parseInt(tv,raw)){ f.indices.clear(); break; }
                int idx=-1; if(!resolveIndex(raw, positions.size(), idx)){ f.indices.clear(); break; }
                f.indices.push_back(idx);
            }
            if(f.indices.size()>=3) faces.push_back(std::move(f));
        }
    }
    extractEdges();
    return !positions.empty();
}

void MeshLoader::extractEdges(){
    edges.clear();
    for(const auto& f: faces){
        size_t n=f.indices.size();
        for(size_t i=0;i<n;++i){
            int a=f.indices[i], b=f.indices[(i+1)%n];
            if(a<0||b<0||a==b) continue;
            if(b<a) std::swap(a,b);
            edges.emplace_back(a,b);
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
}
