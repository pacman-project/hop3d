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

    // Insertion operator
    friend std::ostream& operator<<(std::ostream& os, const PointNormal& point);

    // Extraction operator
    friend std::istream& operator>>(std::istream& is, PointNormal& point);
};

class PointPart{
public:
    /// position
    Vec3 position;
    /// pair: layerNo, realisation/model id
    std::vector<std::pair<int,int>> partsIds;

    ///Construction
    PointPart(){}

    ///Construction
    PointPart(Vec3 _position, std::vector<std::pair<int,int>> _partsIds) : position(_position), partsIds(_partsIds) {}
};

class PointNormalUV : public PointNormal{
public:
    /// image coordinates
    unsigned int u;
    unsigned int v;

    ///Construction
    PointNormalUV(){}

    ///Construction
    PointNormalUV(Vec3 _position, Vec3 _normal, unsigned int _u, unsigned int _v) : PointNormal(_position, _normal), u(_u), v(_v) {}
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

typedef std::vector<hop3d::PointPart> PartsCloud;

typedef std::vector<hop3d::PointNormalUV> PointCloudUV;

typedef std::vector<hop3d::PointColor> PointCloudRGBA;

}

#endif /* DATA_CLOUD_H */
