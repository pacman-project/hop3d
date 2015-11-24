#ifndef DATA_GRAPH_H
#define DATA_GRAPH_H
#include <iostream>
#include "Vocabulary.h"

namespace hop3d {

class Hierarchy {
public:
    /// View dependent layers
    std::vector<VDLayerVocabulary> viewDependentLayers;
    /// View independent layers
    std::vector<VILayerVocabulary> viewIndependentLayers;

    Filter::Seq firstLayer;

    /// Construction
    inline Hierarchy(){}
    /// Construction
    inline Hierarchy(int _viewDepLayersNo, int _viewIndepLayersNo) :
        viewDependentLayers(_viewDepLayersNo), viewIndependentLayers(_viewIndepLayersNo){
    }

    // Insertion operator
    friend std::ostream& operator<<(std::ostream& os, const Hierarchy& hierarchy);

    // Extraction operator
    friend std::istream& operator>>(std::istream& is, Hierarchy& hierarchy);

    /// relation between parts from last view dependent vocabulary and first view-independent
    std::map<int,int> interpreter;

    /// relation between parts second view dependent vocabulary and third view-dependent layers
    std::map<int,int> interpreter2to3;

    /// get normal vector related to the part
    void getNormal(const ViewDependentPart& part, Vec3& normal) const;

    /// get points related to the part assuming that we have flat patches
    void getPoints(const ViewDependentPart& part, std::vector<Vec3>& points) const;

    /// print ids
    void printIds(const ViewDependentPart& part);

    /// Construction
    Hierarchy(std::string configFilename);
};

}

#endif /* DATA_GRAPH_H */
