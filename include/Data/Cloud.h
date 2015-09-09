#ifndef DATA_CLOUD_H
#define DATA_CLOUD_H

#include <iostream>
#include <memory>

#include "Data/Defs.h"
#include "Data/Part.h"

namespace hop3d {

class PointNormal{
public:
    Vec3 position;
    Vec3 normal;
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
    /// Pointernamespace hop3d {

    typedef std::unique_ptr<PartCloud> Ptr;
std::vector<PartRealization> partCloud;

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
