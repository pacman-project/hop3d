#ifndef DATA_CLOUD_H
#define DATA_CLOUD_H

#include <iostream>
#include <memory>

#include "Data/Defs.h"
#include "Data/Part.h"

namespace hop3d {

class PointNormal{
public:
    /// position
    Vec3 position;
    /// normal vector
    Vec3 normal;

    ///Construction
    PointNormal(){}

    ///Construction
    PointNormal(Vec3 _position, Vec3 _normal) : position(_position), normal(_normal) {}
};

class Cloud{
public:
    /// Pointer
    typedef std::unique_ptr<Cloud> Ptr;
    hop3d::U64 ID;
protected:

private:

};

class PartCloud : public Cloud{
public:
    typedef std::unique_ptr<PartCloud> Ptr;

protected:

private:
};

class PointCloud : public Cloud{
public:
    /// Pointer
    typedef std::unique_ptr<PointCloud> Ptr;
    std::vector<hop3d::PointNormal> pointCloudNormal;

protected:

private:

};

}

#endif /* DATA_CLOUD_H */
