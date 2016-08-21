#ifndef DATA_PART_H
#define DATA_PART_H

#include "hop3d/Data/Defs.h"
#include "hop3d/Data/Cloud.h"
//#include "hop3d/Utilities/kabschEst.h"
#include <memory>
#include <vector>
#include <iostream>
#include <set>

namespace hop3d {

template <typename T>
bool isCloseToZero(T x)
{
    return std::abs(x) < std::numeric_limits<T>::epsilon();
}

/// position and normal
typedef std::array<std::array<GaussianSE3,9>,9> PointsSecondLayer;
/// position and normal
typedef std::array<std::array<GaussianSE3,27>,27> PointsThirdLayer;

class ConfigGICP{
public:
    ///verbose
    int verbose;
    /// number of trials with different transformation guess
    int guessesNo;
    /// correspondence distance
    double correspondenceDist;
    /// max Iterations (stop criterion)
    int maxIterations;
    /// transformation Epsilon (stop criterion)
    double transformationEpsilon;
    /// EuclideanFitnessEpsilon (stop criterion)
    double EuclideanFitnessEpsilon;
    /// alpha range
    std::pair<double,double> alpha;
    /// beta range
    std::pair<double,double> beta;
    /// gamma range
    std::pair<double,double> gamma;
};

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
    /// realisation id
    int realisationId;
    /// Type of the part
    Type type;

    /// Ids (index in the vocabulary) of the neighbouring parts from the i-1 layer
    std::array<std::array<int,3>,3> partIds;

    /// Construction
    inline Part() : realisationId(-1){
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                partIds[i][j] = -1;
        id=-1;
    }

    /// Construction
    inline Part(int _id, int _layerId, Type _type) : id(_id), layerId(_layerId), realisationId(-1), type(_type){
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
    /// probability subparts
    typedef std::map<int,double> SubpartsProb;


    /// Position of the part on the image
    ImageCoordsDepth location;

    Vec3 locationEucl;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<Gaussian3D,3>,3> gaussians;

    /// position and normal
    std::array<std::array<GaussianSE3,3>,3> partsPosNorm;

    /// SE3 offset betwen part realization and word from dictionary
    std::array<std::array<Mat34,3>,3> offsets;

    /// probability of subparts
    std::array<std::array<SubpartsProb,3>,3> subpartsProb;

    /// part aggregated from the same level vocabulary
    std::vector<ViewDependentPart> group;

    /// SE3 offset betwen part realization and word from dictionary
    Mat34 offset;

    /// second part (for double surface)
    std::vector<ViewDependentPart> secondVDPart;

    /// Construction
    inline ViewDependentPart() {
        type = PART_VIEW_DEP;
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                offsets[i][j]=Mat34::Identity();
            }
        }
    }

    /// Construction
    inline ViewDependentPart(int _id, int _layerId, ImageCoordsDepth _location) :
        Part(_id, _layerId, PART_VIEW_DEP), location(_location){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                offsets[i][j]=Mat34::Identity();
            }
        }
    }

    ///find optimal transformation between normals
    static double findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& transOpt, int& rotId);

    ///find optimal transformation between normals
    static double findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& vocabulary, int distanceMetric, Mat34& transOpt, int& rotId);

    ///find optimal transformation between normals
    static double findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, int distanceMetric, Mat34& transOpt, int& rotId);

    /// create point cloud from second layer part
    static int createPointsMatrix(const ViewDependentPart& part, const ViewDependentPart::Seq& vocabulary, int rotIndex, PointsSecondLayer& points, Eigen::MatrixXd& partIds);

    /// create point cloud from second layer part
    static int createPointsMatrix(const ViewDependentPart& part, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, int rotIndex, PointsThirdLayer& points, Eigen::MatrixXd& partIds);

    /// find SE3 transformation
    static bool findSE3Transformation(const PointsSecondLayer& pointsA, const PointsSecondLayer& pointsB, Mat34& trans);

    /// find SE3 transformation
    static bool findSE3Transformation(const PointsThirdLayer& pointsA, const PointsThirdLayer& pointsB, Mat34& trans);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters, int distanceMetric);

    /// compute distance between view dependent parts
    static double distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& estimatedTransform, int& rotId);

    /// compute distance between view dependent parts (invariant version)
    static double distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, const ViewDependentPart::Seq& vocabulary, Mat34& estimatedTransform, int& rotId);

    /// compute distance between view dependent parts (invariant version) for fourth layer
    static double distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, Mat34& estimatedTransform, int& rotId);

    /// compute distance between view dependent parts
    static double distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters, int distanceMetric);

    /// view invariant error for two parts with known SE3 transformation
    static double computeError(const ViewDependentPart& partA, const ViewDependentPart& partB, const Mat34& transformation, int type, double coeff);

    /// view invariant error for two second layer parts with known SE3 transformation
    static double computeError(const PointsSecondLayer& partA, const PointsSecondLayer& partB, const Eigen::MatrixXd& partAids, const Eigen::MatrixXd& partBids, const Mat34& transformation, int type, double coeff);

    /// view invariant error for two second layer parts with known SE3 transformation
    static double computeError(const PointsThirdLayer& partA, const PointsThirdLayer& partB, const Eigen::MatrixXd& partAids, const Eigen::MatrixXd& partBids, const Mat34& transformation, int type, double coeff);

    ///get normal vector related to that part
    void getNormal(Vec3& normal, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters) const;

    /// remove elements which belong to "second surface"
    static bool removeSecondSurface(ViewDependentPart& part, double distThreshold);

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
        /// realisation id
        int realisationId;

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
    /// point cloud
    hop3d::PointCloud cloud;
    /// incoming ids
    std::set<int> incomingIds;
    /// realisation id
    int realisationId;

    /// id of neighbouring parts
    std::array<std::array<std::array<int,3>,3>,3> partIds;

    /// relative positions of neighbouring parts
    std::array<std::array<std::array<Mat34,3>,3>,3> neighbourPoses;

    /// relative positions of neighbouring parts
    std::array<std::array<std::array<hop3d::PointCloud,3>,3>,3> clouds;

    /// statistics for positions
    std::array<std::array<std::array<GaussianSE3,3>,3>,3> gaussians;

    /// SE3 offset betwen part realization and word from dictionary
    std::array<std::array<std::array<Mat34,3>,3>,3> offsets;

    ViewIndependentPart() : pose(Mat34::Identity()), offset(Mat34::Identity()){
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				for (int k=0;k<3;k++)
					partIds[i][j][k] = -1;
        id=-1; realisationId=-1;
    }

    ViewIndependentPart(int size) : pose(Mat34::Identity()), offset(Mat34::Identity()){
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				for (int k=0;k<3;k++)
					partIds[i][j][k] = -1;
        size=size-1; id=-1;
        realisationId=-1;
    }// required by octree

    /// Print
    void print() const;

    /// compute coordinate system from normal vector
    static Mat33 coordinateFromNormal(const Vec3& _normal);

    /// compute distance between view-independent parts
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, Mat34& offset);

    /// compute distance between view-independent parts
    static double distanceGICP(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ConfigGICP& configGICP, Mat34& offset);

    /// compute distance between view-independent parts
    static double distanceUmeyama(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, Mat34& offset);

    /// compute distance between view-independent parts
    static double distanceUmeyama(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, const ViewIndependentPart::Seq& vocabulary, Mat34& offset);

    ///find optimal transformation for view independent parts
    static double findOptimalTransformation(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, Mat34& transOpt);

    ///find optimal transformation for view independent parts
    static double findOptimalTransformation(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq& vocabulary, int distanceMetric, Mat34& transOpt);

    /// compute distance between view-independent parts
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary, Mat34& offset);

    /// compute distance between view-independent parts (7th layer)
    static double distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary1, const ViewIndependentPart::Seq vocabulary2, Mat34& offset);

    /// compute error between tow point clous
    static double computeError(const hop3d::PointCloud& cloudA, const hop3d::PointCloud& cloudB, hop3d::Mat34& trans);

    /// normalize vector
    /*static inline void normalizeVector(Vec3& normal){
        double norm = normal.norm();
        normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
    }*/

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
    /// find rotated coordinates
    static void findCorrespondence(int idX, int idY, int idZ, int rotX, int rotY, int rotZ, std::array<int,3>& newCoords);
    /// compute error between tow point clous
    static double computeError(const ViewIndependentPart& partA, const ViewIndependentPart& partB, hop3d::Mat34& trans, int type, double coeff);
    /// Compute distance between two parts
    static double computeVIPartsDistance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, int distanceMetric, Mat34& transOpt);
    /// Compute distance between two parts
    static double computeVIPartsDistance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, const ViewIndependentPart::Seq vocabulary, int distanceMetric, Mat34& transOpt);
    /// Get corresponding points from parts
    static void getCorrespondingPoints(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, std::vector<Vec3>& setA, std::vector<Vec3>& setB);
};

}
#endif /* DATA_PART_H */
