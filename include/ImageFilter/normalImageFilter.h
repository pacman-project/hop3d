/** @file normalImageFilter.h
 *
 * implementation - depth image filter (normal)
 *
 */

#ifndef NORMAL_IMAGE_FILTER_H_INCLUDED
#define NORMAL_IMAGE_FILTER_H_INCLUDED

#include "imageFilter.h"
#include "Utilities/Reader.h"
#include "Utilities/Writer.h"
#include "../../external/tinyXML/tinyxml2.h"
#include "Utilities/ImagesDisplay.h"
#include "Utilities/depthSensorModel.h"

namespace hop3d {
    /// create a single depth image filter
    ImageFilter* createNormalImageFilter(void);
    /// create a single depth image filter
    ImageFilter* createNormalImageFilter(std::string config, std::string sensorConfig);


/// Image filter implementation
class NormalImageFilter: public ImageFilter {
    /// response: id, response value
    typedef std::pair<int,double> Response;
    /// 2D array of octets
    typedef std::vector< std::vector<Octet> > OctetsImage;
    /// 2D array of vocabularies
    typedef std::vector< std::vector<ViewDependentPart> > PartsImage;
public:
    /// Pointer
    typedef std::unique_ptr<NormalImageFilter> Ptr;

    /// Construction
    NormalImageFilter(void);

    /// Construction
    NormalImageFilter(std::string config, std::string sensorConfig);

    /// Destruction
    ~NormalImageFilter(void);

    /// Name of the map
    const std::string& getName() const;

    ///compute set of octets from set of the depth images
    void computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets);

    /// compute set of octets from set of the ids image
    void getOctets(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, Octet::Seq& octets);

    /// get filters
    void getFilters(Filter::Seq& _filters) const;

    /// not used here
    void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName);

    /// define 2rd layer octet images using selected words from third layer
    void computeImages3rdLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary);

    /// get last view dependent layer parts from the image
    void getLastVDLayerParts(int categoryNo, int objectNo, int imageNo, std::vector<ViewDependentPart>& parts) const;

    /// compute coordinate system from normal vector
    static Mat33 coordinateFromNormal(const Vec3& _normal);

    /// get set of ids for the given input point
    void getPartsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            /// number of rings -- number of filters: 2*((1-2^ringsNo)/(1-2))
            int ringsNo;
            /// filter size in pixels
            int filterSize;
            /// displaying variables and images for debugging
            int verbose;
            /// depth sensor model filenames
            std::string sensorFilename;
            /// compute response if points no in filter window is bigger than threshold
            int PCAbackgroundThreshold;
            /// true fin max for the group, false - find max for the whole window
            bool nonMaximumSupressionGroup;
            ///minimal number of elements in the octet
            int minOctetSize;
            /// use median filter
            bool useMedianFilter;
            /// kernel size
            int kernelSize;
            /// PCA window size
            int PCAWindowSize;
            /// PCA max distance between points -- if bigger run clusterization
            double PCADistThreshold;
            /// use clustering
            bool PCAuseClustering;
            /// create two clusters and check distance between them. If distance is PCARelDistClusters times larger than width of the front cluster run PCA
            double PCARelDistClusters;
    };

private:
    ///Configuration of the module
    Config config;
    ///Loaded filters
    hop3d::Filter::Seq filters;
    /// sensor model
    DepthSensorModel sensorModel;
    /// octets images
    std::vector<std::vector< std::vector<OctetsImage>>> octetsImages1stLayer;
    /// octets images -- second layer
    std::vector<std::vector< std::vector<OctetsImage>>> octetsImages2ndLayer;
    /// parts images
    std::vector<std::vector< std::vector<PartsImage>>> partsImages;

    /// discretization: convert normal vector to id
    int toId(const Vec3& normal) const;

    /// convert id of the filter to normal vector
    void toNormal(int id, Vec3& normal) const;

    /// compute normal on the image (filter size)
    void computeNormal(int u, int v, std::vector< std::vector<hop3d::PointNormal> >& cloudOrd);

    /// Compute normal vector using PCA
    void normalPCA(std::vector<hop3d::PointNormal>& points, hop3d::PointNormal& pointNormal);

    /// Generate filters
    void generateFilters(void);

    /// normalize vector
    static void normalizeVector(Vec3& normal);

    ///extract octets from response image
    OctetsImage extractOctets(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, hop3d::Octet::Seq& octets);

    /// compute otet for given location on response image
    bool computeOctet(const std::vector< std::vector<Response> >& responseImg,  const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u,  int v, Octet& octet) const;

    /// compute max response in vindow
    bool findMaxResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const;

    /// compute max response for the most numerous group in the window
    bool findMaxGroupResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const;

    /// Fill in octet
    void fillInOctet(const OctetsImage& octetsImage, const ViewDependentPart::Seq& dictionary, int u, int v, Octet& octet) const;

    //set relative position for octets
    void computeRelativePositions(Octet& octet, int layerNo) const;

    /// determine id of the part using dictionary
    int findId(const ViewDependentPart::Seq& dictionary, const Octet& octet) const;

    /// update structure which holds octets images
    void updateOctetsImages1stLayer(int categoryNo, int objectNo, int imageNo, const OctetsImage& octetsImage);

    /// update structure which holds octets images
    void updateOctetsImages2ndLayer(int categoryNo, int objectNo, int imageNo, const OctetsImage& octetsImage);

    /// update structure which holds parts images
    void updatePartsImages(int categoryNo, int objectNo, int imageNo, const PartsImage& partsImage);

    /// Apply median filter on the image
    void medianFilter(const cv::Mat& inputImg, cv::Mat& outputImg, int kernelSize);

    /// compute median
    uint16_t median(const cv::Mat& inputImg, int u, int v, int kernelSize);

    /// try to extract two clusters. If clusters are well separated (PCARelDistClusters parameter) return true
    bool extractGroup(const std::vector<hop3d::PointNormal>& points, std::vector<hop3d::PointNormal>& pointGroup) const;
};

}
#endif // NORMAL_IMAGE_FILTER_H_INCLUDED
