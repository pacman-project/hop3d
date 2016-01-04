#ifndef DATA_CLOUD_H
#define DATA_CLOUD_H

#include <iostream>
#include <memory>
#include <array>

#include "hop3d/Data/Defs.h"

namespace hop3d {

class PointNormal{
public:
    /// position
    Vec3 position;
    /// normal vector
    Vec3 normal;

    /// Euclidean distance
    double euclideanDistance(const Vec3& vec) const;

    ///Construction
    PointNormal(){}

    ///Construction
    PointNormal(Vec3 _position, Vec3 _normal) : position(_position), normal(_normal) {}
};

class PointColor{
public:
    /// position
    Vec3 position;
    /// normal vector
    std::array<double,4> color;

    ///Construction
    PointColor(){}

    ///Construction
    PointColor(Vec3 _position, std::array<double,4> _color) : position(_position), color(_color) {}
};

typedef std::vector<hop3d::PointNormal> PointCloud;

typedef std::vector<hop3d::PointColor> PointCloudRGBA;

}

#endif /* DATA_CLOUD_H */
