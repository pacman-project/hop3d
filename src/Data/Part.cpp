#include "Data/Part.h"
#include "Data/Vocabulary.h"

namespace hop3d {

/// Print
void ViewDependentPart::print() const{
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Image coords: " << location.u << ", " << location.v << "\n";
    std::cout << "Group size: " << group.size() << "\n";
    std::cout << "ids: ";
    for (int i =0; i<3; i++){
        for (int j =0; j<3; j++){
            std::cout << this->partIds[i][j] << ", ";
        }
    }
    std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++)
        std::cout << it->id << ", ";
    std::cout << "\n";
    /*for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i-1 << ", " << j-1 << "): ";
            std::cout << gaussians[i][j].mean.transpose() << "\n";
            std::cout << "covariance(" << i-1 << ", " << j-1 << "):\n";
            std::cout << gaussians[i][j].covariance << "\n";
        }
    }*/
}

/// Print
void ViewIndependentPart::print() const{
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Group size: " << group.size() << "\n";
    std::cout << "ids: ";
    for (int i =0; i<3; i++){
        for (int j =0; j<3; j++){
            std::cout << this->partIds[i][j] << ", ";
        }
    }
    std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++)
        std::cout << *it << ", ";
    std::cout << "\n";
    /*for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i-1 << ", " << j-1 << "): ";
            std::cout << gaussians[i][j].mean.transpose() << "\n";
            std::cout << "covariance(" << i-1 << ", " << j-1 << "):\n";
            std::cout << gaussians[i][j].covariance << "\n";
        }
    }*/
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++)
        for (size_t j=0; j<partA.partIds.size();j++){
            if ((partA.partIds[i][j]==-1)&&(partB.partIds[i][j]==-1))
                sum+=0;
            else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
                sum+=1;
            }
            else
                sum+=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
        }
    return sum;
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++)
        for (size_t j=0; j<partA.partIds.size();j++)
            if (partA.layerId==3){
                sum+=distance(layer2vocabulary[partA.partIds[i][j]], layer2vocabulary[partB.partIds[i][j]], filters);
            }
    return sum;
}

}
