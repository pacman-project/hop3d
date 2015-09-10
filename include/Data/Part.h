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
    /// Part type
    enum Type {
        /// view dependent
        PART_VIEW_DEP,
        /// view independent
        PART_VIEW_INDEP,
    };

    /// Part id
    int id;
    /// Layer id
    int layerId;
    /// Type of the part
    Type type;

    /// Id of the neighbouring parts from the same layer
    std::array<std::array<int,3>,3> partIds;

    /// OR parts -- aggregated parts at the same layer
    std::vector<int> ORparts;

    /// Construction
    inline Part(int _id, int _layerId, Type _type) : id(_id), layerId(_layerId), type(_type){
    }
};

class ViewDependentPart: Part{
public:
    /// Sequence
    typedef std::vector<ViewDependentPart> Seq;
    /// Pointer
    typedef std::unique_ptr<ViewDependentPart> Ptr;

    /// Position of the part on the image
    ImageCoords location;

    /// Camera pose id
    int cameraPoseId;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian2D,3>,3> gaussians;

    /// Construction
    inline ViewDependentPart(int _id, int _layerId, Type _type, ImageCoords _location) :
        Part(_id, _layerId, _type), location(_location){
    }
};

class ViewIndependentPart: Part{
public:
    /// Sequence
    typedef std::vector<ViewIndependentPart> Seq;
    /// Pointer
    typedef std::unique_ptr<ViewIndependentPart> Ptr;

    /// Pose of the part in 3D space
    Mat34 pose;

    /// Construction
    inline ViewIndependentPart(int _id, int _layerId, Type _type, Mat34 _pose) :
        Part(_id, _layerId, _type), pose(_pose){
    }
};

}
#endif /* DATA_PART_H */
