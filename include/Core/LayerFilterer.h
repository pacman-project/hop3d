#ifndef CORE_LAYERFILTERER_H
#define CORE_LAYERFILTERER_H
#include <iostream>
#include <memory>
#include "Data/Cloud.h"
#include <flann/flann.hpp>

class LayerFilterer{

public:
    /// Pointer
    typedef std::unique_ptr<LayerFilterer> Ptr;
    int nearestNeighbour(const PointCloud& inputPointCloud);
    template<typename T>
    void loadPointCloud(flann::Matrix<T>& dataset, const PointCloud &inputPointCloud);
protected:

private:

};


#endif /* CORE_LAYERFILTERER_H */
