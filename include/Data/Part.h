#ifndef DATA_PART_H
#define DATA_PART_H

#include "Data/Defs.h"
#include "Utilities/kabschEst.h"

#include <memory>
#include <vector>
#include <iostream>

namespace hop3d {

template <typename T>
bool isCloseToZero(T x)
{
    return std::abs(x) < std::numeric_limits<T>::epsilon();
}

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
    inline Part(){
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                partIds[i][j] = -1;
    }

    /// Construction
    inline Part(int _id, int _layerId, Type _type) : id(_id), layerId(_layerId), type(_type){
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                partIds[i][j] = -1;
        id=-1;
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

    Vec3 locationEucl;

    /// Camera pose id
    int cameraPoseId;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian3D,3>,3> gaussians;

    /// position and normal
    std::array<std::array<GaussianSE3,3>,3> partsPosNorm;

    /// part aggregated from the same level vocabulary
    std::vector<ViewDependentPart> group;

    /// Construction
    inline ViewDependentPart(){
        type = PART_VIEW_DEP;
    }

    /// Construction
    inline ViewDependentPart(int _id, int _layerId, ImageCoordsDepth _location) :
        Part(_id, _layerId, PART_VIEW_DEP), location(_location){
    }

    ///find optimal transformation between normals
    static double findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& transOpt);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters, int distanceMetric);

    /// compute distance between view dependent parts
    static double distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& estimatedTransform);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters, int distanceMetric);

    /// view invariant error for two parts with known SE3 transformation
    static double computeError(const ViewDependentPart& partA, const ViewDependentPart& partB, const Mat34& transformation, int type, double coeff);

    ///get normal vector related to that part
    void getNormal(Vec3& normal, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters) const;

    /// remove elements which belong to "second surface"
    static bool removeSecondSurface(ViewDependentPart& part);

    /// Insertion operator
    friend std::ostream& operator<<(std::ostream& os, const ViewDependentPart& part);

    /// Extraction operator
    friend std::istream& operator>>(std::istream& is, ViewDependentPart& part);

    /// check if part is background
    bool isBackground(void) const;

    /// Print
    void print() const;
};

class ViewIndependentPart{
public:
    /// Sequence
    typedef std::vector<ViewIndependentPart> Seq;

    class Part3D{
    public:
        /// pose of the part
        Mat34 pose;
        /// id of the part
        int id;

        Part3D(void){}

        Part3D(Mat34& _pose, int _id) : pose(_pose), id(_id){}
        /// Insertion operator
        friend std::ostream& operator<<(std::ostream& os, const Part3D& part);

        /// Extraction operator
        friend std::istream& operator>>(std::istream& is, Part3D& part);
    };

    /// part composition
    std::vector<Part3D> parts;

    /// pose
    Mat34 pose;
    /// offset for part realization (needed for reconstruction)
    Mat34 offset;
    /// Part id
    int id;
    /// Layer id
    int layerId;
    /// part aggregated from the same level vocabulary
    std::vector<ViewIndependentPart> group;

    /// id of neighbouring parts
    std::array<std::array<std::array<int,3>,3>,3> partIds;

    /// relative positions of neighbouring parts
    std::array<std::array<std::array<Mat34,3>,3>,3> neighbourPoses;

    /// statistics for positions
    std::array<std::array<std::array<GaussianSE3,3>,3>,3> gaussians;

    ViewIndependentPart() : pose(Mat34::Identity()), offset(Mat34::Identity()){
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				for (int k=0;k<3;k++)
					partIds[i][j][k] = -1;
        id=-1;
    }
    ViewIndependentPart(int size) : pose(Mat34::Identity()), offset(Mat34::Identity()){
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				for (int k=0;k<3;k++)
					partIds[i][j][k] = -1;
        size=size-1; id=-1;
    }// required by octree
    /// Print
    void print() const;

    /// compute distance between view-independent parts
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, Mat34& offset);

    /// compute distance between view-independent parts
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary, Mat34& offset);

    /// compute distance between view-independent parts (7th layer)
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary1, const ViewIndependentPart::Seq vocabulary2, Mat34& offset);

    /// normalize vector
    /*static inline void normalizeVector(Vec3& normal){
        double norm = normal.norm();
        normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
    }*/

    /// compute coordinate system from normal vector
    static Mat33 coordinateFromNormal(const Vec3& _normal);
    /// compute the min distance to the set of parts
    static double nearestNeighbour(const Mat34& pose, std::vector<std::pair<Mat34, int>> parts, int& neighbourId);

    /// Insertion operator
    friend std::ostream& operator<<(std::ostream& os, const ViewIndependentPart& part);

    /// Extraction operator
    friend std::istream& operator>>(std::istream& is, ViewIndependentPart& part);
private:
    ///compute offset for two sets of points
    static Mat34 computeOffset(const std::vector<std::pair<Mat34, int>>& partASeq, const std::vector<std::pair<Mat34, int>>&partBSeq, Vec3& meanPosA, Vec3& normA, Vec3& meanPosB, Vec3& normB);
    ///get central points from parts, mean and mean normal vector
    static void getPoints(const ViewIndependentPart&partA, const ViewIndependentPart& partB, std::vector<std::pair<Mat34, int>>& partASeq, std::vector<std::pair<Mat34, int>>& partBSeq, Vec3& meanPosA, Vec3& normA, Vec3& meanPosB, Vec3& normB);
};

}
#endif /* DATA_PART_H */
