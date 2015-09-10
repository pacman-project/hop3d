#ifndef DATA_PART_H
#define DATA_PART_H
#include <iostream>
#include <memory>
#include <vector>

#include "Data/Defs.h"

namespace hop3d {

class Part{
public:
    /// Sequence
    typedef std::vector<Part> Seq;
    /// Pointer
    typedef std::unique_ptr<Part> Ptr;

    /// Part id
    int id;
    /// Layer id
    int layerId;
};

class ViewDependentPart : Part{
public:
    /// Sequence
    typedef std::vector<ViewDependentPart> Seq;
    /// Pointer
    typedef std::unique_ptr<ViewDependentPart> Ptr;

    /// Position of the part on the image
    ImageCoords location;

    /// Id of the neighbouring parts from the same layer
    std::array<std::array<int,3>,3> partIds;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian2D,3>,3> gaussians;

    /// OR parts -- aggregated parts at the same layer
    std::vector<int> ORparts;
};


typedef std::vector<ViewDependentPart> LayerVocabulary;

class PartRealization : public Part {

public:
    /// Pointer
    typedef std::unique_ptr<PartRealization> Ptr;

    hop3d::Vec3 position;
    //hop3d::ReferenceFrame referenceFrame;
    hop3d::F32 activation;
    hop3d::I8 scale;
    typedef std::vector<PartRealization>  Seq;

protected:

private:

};

}
#endif /* DATA_PART_H */
