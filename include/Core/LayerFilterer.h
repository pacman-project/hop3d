#ifndef CORE_LAYERFILTERER_H
#define CORE_LAYERFILTERER_H
#include <iostream>
#include <memory>
#include <chrono>
#include "Data/Cloud.h"
#include "Data/Part.h"
#include <flann/flann.hpp>

class LayerFilterer{

public:
    /// Pointer
    typedef std::unique_ptr<LayerFilterer> Ptr;
    int nearestNeighbour(const hop3d::PointCloud& inputPointCloud, int nearestNeigbours = 4 );
    int radiusSearch(const hop3d::PointCloud &inputPointCloud, float radius);
    int FindActivations();

protected:
    template<typename T>
    void loadPointCloud(flann::Matrix<T>& dataset, const hop3d::PointCloud &inputPointCloud);
private:
    hop3d::PartRealization partRealization;
};


#endif /* CORE_LAYERFILTERER_H */
