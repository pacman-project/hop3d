#ifndef DATA_GRAPH_H
#define DATA_GRAPH_H
#include <iostream>
#include "Vocabulary.h"
#include <cstdint>
#include <map>

namespace hop3d {

class Hierarchy {
public:
    /// View dependent layers
    std::vector<VDLayerVocabulary> viewDependentLayers;
    /// View independent layers
    std::vector<VILayerVocabulary> viewIndependentLayers;

    typedef std::vector<std::uint32_t> IndexSeq;
    /** Index sequence map */
    typedef std::map<std::uint32_t, IndexSeq> IndexSeqMap;

    typedef std::set<std::uint32_t> IndexSet;
    /** Index sequence map */
    typedef std::map<std::uint32_t, IndexSet> IndexSetMap;

    /// first layer -- filters
    Filter::Seq firstLayer;
    /// create view dependent layers from words of i-th layer
    int viewDepPartsFromLayerNo;

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
    //std::map<int,int> interpreter2to3;

    /// get normal vector related to the part
    void getNormal(const ViewDependentPart& part, Vec3& normal) const;

    /// get points related to the part assuming that we have flat patches
    //void getPoints(const ViewDependentPart& part, std::vector<Vec3>& points) const;

    /// print ids
    void printIds(const ViewDependentPart& part);

    /// compute mean vector for 2nd layer part
    void computeMeanVector(const ViewDependentPart& part, Vec3& normal) const;

    /// compute graph from hierarchy structure
    void computeGraph(IndexSeqMap& hierarchyGraph);

    /// add parts to existing clusters
    void addParts(const ViewDependentPart::Seq& parts, int layerNo);

    /// add parts to existing clusters
    void addParts(const ViewIndependentPart::Seq& parts, int layerNo);

    /// add parts (new clusters)
    void addNewParts(const ViewDependentPart::Seq& parts, int layerNo);

    /// add parts (new clusters)
    void addNewParts(const ViewIndependentPart::Seq& parts, int layerNo);

    /// show info
    void showInfo(void);

    /// compute statisticts (results are written to representative parts)
    void computeStats(void);

    /// compute statisticts for selected layer (results are written to representative parts)
    void computeVDStats(int layerNo);

    /// reconstruct part
    void reconstructPart(ViewDependentPart& _part, size_t layerNo) const;

    /// Construction
    Hierarchy(std::string configFilename);

private:
    IndexSeqMap graph;
};

}

#endif /* DATA_GRAPH_H */
