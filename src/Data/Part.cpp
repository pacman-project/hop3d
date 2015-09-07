#include "Data/Part.h"

int hop3d::Part::setPartId(hop3d::U64 id)
{
    Id = id;
    return 0;
}

hop3d::U64 hop3d::Part::getPartId() const
{
    return Id;
}




int hop3d::FirstLayerPart::setNormal(hop3d::Vec3 normal)
{
    this->Normals = normal;
    return 0;
}

hop3d::Vec3 hop3d::FirstLayerPart::getNormal() const
{
    return this->Normals;
}


