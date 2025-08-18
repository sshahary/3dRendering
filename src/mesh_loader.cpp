#include "mesh_loader.hpp"
#include <fstream>
#include <sstream>

bool MeshLoader::loadOBJ(const std::string& path){
    positions.clear(); faces.clear(); edges.clear();
    std::ifstream is(path);
    if(!is) return false;
    std::string line;
    while(std::getline(is,line)){
        if(line.empty() || line[0]=='#') continue;
        std::istringstream ss(line);        std::string tag; ss>>tag;
        if(tag=="v"){ float x,y,z; ss>>x>>y>>z; positions; }
        else if(tag=="f"){
            PolyFace f; std::string tok;
            while(ss>>tok){
                size_t s = tok.find('/'); if(s!=std::string::npos) tok = tok.substr(0,s);
                int idx = std::stoi(tok) - 1; f.indices.push_back(idx);
            }
            if(f.indices.size()>=3) faces.push_back(f);
        }
    }
    extractEdges();
    return !positions.empty();
}

void MeshLoader::extractEdges(){
    for(const auto& f: faces){
        for(size_t i=0;i<f.indices.size();++i){
            int a = f.indices[i];
            int b = f.indices[(i+1)%f.indices.size()];
            if(a>=0 && b>=0) edges.emplace_back(a,b);
        }
    }
}
