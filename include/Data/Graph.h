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

    /// Construction
    inline Hierarchy(int _viewDepLayersNo, int _viewIndepLayersNo) :
        viewDependentLayers(_viewDepLayersNo), viewIndependentLayers(_viewIndepLayersNo){
    }
    /// Construction
    Hierarchy(std::string configFilename);
};

}

#endif /* DATA_GRAPH_H */
