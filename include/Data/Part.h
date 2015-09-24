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

    /// Ids (index in the vocabulary) of the neighbouring parts from the i-1 layer
    std::array<std::array<int,3>,3> partIds;

    /// Construction
    inline Part(){};

    /// Construction
    inline Part(int _id, int _layerId, Type _type) : id(_id), layerId(_layerId), type(_type){
    }
};

class ViewDependentPart : public Part{
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
    std::array<std::array<Gaussian3D,3>,3> gaussians;

    /// part aggregated from the same level vocabulary
    std::vector<ViewDependentPart> group;

    /// Construction
    inline ViewDependentPart(){};

    /// Construction
    inline ViewDependentPart(int _id, int _layerId, ImageCoords _location) :
        Part(_id, _layerId, PART_VIEW_DEP), location(_location){
    }

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters);
    /// Print
    void print() const;
};

class ViewIndependentPart : public Part{
public:
    /// Sequence
    typedef std::vector<ViewIndependentPart> Seq;
    /// Pointer
    typedef std::unique_ptr<ViewIndependentPart> Ptr;

    /// Pose of the part in 3D space
    Mat34 pose;

    /// id of parts from last view-dependent layer which create current part
    std::vector<int> group;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian3D,3>,3> gaussians;

    /// Construction
    inline ViewIndependentPart(){};

    /// Construction
    inline ViewIndependentPart(int _id, int _layerId, Mat34 _pose) :
        Part(_id, _layerId, PART_VIEW_INDEP), pose(_pose){
    }
    /// Print
    void print() const;
};

}
#endif /* DATA_PART_H */
