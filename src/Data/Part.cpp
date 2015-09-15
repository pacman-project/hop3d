#include "Data/Part.h"

namespace hop3d {

/// Print
void ViewDependentPart::print(){
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Image coords: " << location.u << ", " << location.v << "\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i-1 << ", " << j-1 << "):\n";
            std::cout << gaussians[i][j].mean << "\n";
            std::cout << "covariance(" << i-1 << ", " << j-1 << "):\n";
            std::cout << gaussians[i][j].covariance << "\n";
        }
    }
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++)
        for (size_t j=0; j<partA.partIds.size();j++)
            if (partA.layerId==2)
                sum+=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
    return sum;
}
}
