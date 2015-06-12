#ifndef DATA_CLOUD_H
#define DATA_CLOUD_H
#include "Data/Defs.h"
#include "Data/Part.h"

class Cloud{

public:
hop3d::U64 ID;
protected:

private:

};

class PartCloud : public Cloud{
public:
std::vector<PartRealization> partCloud;

protected:

private:
};

class PointCloud : public Cloud{

public:
std::vector<hop3d::PointNormal> PointCloudNormal;


protected:

private:

};


#endif /* DATA_CLOUD_H */
