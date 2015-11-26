#ifndef DATA_CLOUD_H
#define DATA_CLOUD_H

#include <iostream>
#include <memory>
#include <array>

#include "Data/Defs.h"
#include "Data/Part.h"

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

class Cloud{
public:
    /// Pointer
    typedef std::unique_ptr<Cloud> Ptr;
    int id;
protected:

private:

};

class PointCloud{
public:
    /// Pointer
    typedef std::unique_ptr<PointCloud> Ptr;
    std::vector<hop3d::PointNormal> pointCloudNormal;

protected:

private:

};

class PointCloudRGBA{
public:
    /// Pointer
    typedef std::unique_ptr<PointCloudRGBA> Ptr;
    std::vector<hop3d::PointColor> pointCloudRGBA;
protected:
private:
};

}

#endif /* DATA_CLOUD_H */
