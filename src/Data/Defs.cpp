#include "Data/Defs.h"

namespace hop3d {

    /// compute distance between filters -- dot product for normals
    double Filter::distance (const Filter& filterA, const Filter& filterB){
        return (filterA.id==filterB.id) ? 0 : (double)(filterA.normal.adjoint()*filterB.normal);
    }

    /// Print octet
    void Octet::print() const{
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

    /// compute distance between octets -- dot product for normals for each filter
    double Octet::distance(const Octet& octetA, const Octet& octetB, const Filter::Seq& filters){
        if (filters.size()==0){
            std::cout << "Empty filters set!\n"; getchar();
            return std::numeric_limits<double>::max();
        }
        if (octetA.filterIds==octetB.filterIds){//fast
            return 0;
        }
        double sum=0;
        for (size_t i=0; i<octetA.filterIds.size();i++)
            for (size_t j=0; j<octetA.filterIds.size();j++)
                sum+=Filter::distance(filters[octetA.filterIds[i][j]], filters[octetB.filterIds[i][j]]);
        return sum;
    }
}
