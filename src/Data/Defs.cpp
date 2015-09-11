#include "Data/Defs.h"

namespace hop3d {

    /// Print octet
    void Octet::print(){
        std::cout << "Camera pose id:" << poseId << "\n";
        std::cout << "Filter ids:\n";
        for (size_t i=0;i<filterIds.size();i++){
            for (size_t j=0;j<filterIds[i].size();j++){
                std::cout << filterIds[i][j] << ", ";
            }
            std::cout << "\n";
        }
        std::cout << "Filter poses (u,v,depth):\n";
        for (size_t i=0;i<filterPos.size();i++){
            for (size_t j=0;j<filterPos[i].size();j++){
                std::cout << "(" << filterPos[i][j].u << "," << filterPos[i][j].v << "," << filterPos[i][j].depth << "), ";
            }
            std::cout << "\n";
        }
    }

}
