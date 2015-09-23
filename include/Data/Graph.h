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

    /// get normal vector related to the part
    void getNormal(const ViewDependentPart& part, Vec3& normal) const;

    /// Construction
    Hierarchy(std::string configFilename);
};

}

#endif /* DATA_GRAPH_H */
