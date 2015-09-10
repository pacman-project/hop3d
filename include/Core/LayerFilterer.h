#ifndef CORE_LAYERFILTERER_H
#define CORE_LAYERFILTERER_H
#include <iostream>
#include <memory>
#include <chrono>
#include "Data/Cloud.h"
#include "Data/Part.h"
#include "Data/Vocabulary.h"
#include "Utilities/Reader.h"
#include <flann/flann.hpp>

class LayerFilterer{

public:
    /// Pointer
    typedef std::unique_ptr<LayerFilterer> Ptr;
    int nearestNeighbour(const hop3d::PointCloud& inputPointCloud, int nearestNeigbours = 4 );
    int radiusSearch(const hop3d::PointCloud &inputPointCloud, double radius);
    int findActivations(const hop3d::PointCloud &inputPointCloud);
    int readFirstLayerVocabulary(std::string fileName);

protected:
    template<typename T>
    void loadPointCloud(flann::Matrix<T>& dataset, const hop3d::PointCloud &inputPointCloud);
private:
    //hop3d::PartRealization::Seq partRealizations;
    //hop3d::LayerVocabulary first;
    hop3d::Reader reader;

};


#endif /* CORE_LAYERFILTERER_H */
