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
    inline Part(){}

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
    ImageCoordsDepth location;

    /// Camera pose id
    int cameraPoseId;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian3D,3>,3> gaussians;

    /// part aggregated from the same level vocabulary
    std::vector<ViewDependentPart> group;

    /// Construction
    inline ViewDependentPart(){};

    /// Construction
    inline ViewDependentPart(int _id, int _layerId, ImageCoordsDepth _location) :
        Part(_id, _layerId, PART_VIEW_DEP), location(_location){
    }

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters);

    ///get normal vector related to that part
    void getNormal(Vec3& normal, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters) const;

    /// Print
    void print() const;
};

class ViewIndependentPart{
public:
    class Part3D{
    public:
        /// pose of the part
        Mat34 pose;
        /// id of the part
        int id;

        Part3D(void){};
        Part3D(Mat34& _pose, int _id) : pose(_pose), id(_id){};
    };

    /// part composition
    std::vector<Part3D> parts;

    /// pose
    Mat34 pose;
    /// Part id
    int id;
    /// Layer id
    int layerId;
    /// part aggregated from the same level vocabulary
    std::vector<int> group;

    /// id of neighbouring parts
    std::array<std::array<std::array<int,3>,3>,3> partIds;

    /// relative positions of neighbouring parts
    std::array<std::array<std::array<Mat34,3>,3>,3> neighbourPoses;

    ViewIndependentPart(): partIds{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1} {
        id=-1;
    }
    ViewIndependentPart(int size) : partIds{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}{
        size=size-1; id=-1;
    }// required by octree
    /// Print
    void print() const;
};

}
#endif /* DATA_PART_H */
