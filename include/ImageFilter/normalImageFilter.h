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


/// Map implementation
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
    void computeOctets(const cv::Mat& depthImage, hop3d::Octet::Seq& octets);

    /// compute set of octets from set of the ids image
    void getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets);

    /// get filters
    void getFilters(Filter::Seq& _filters) const;

    /// not used here
    void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName);

    /// define 2rd layer octet images using selected words from third layer
    void computeImages3rdLayer(const ViewDependentPart::Seq& dictionary);

    /// get last view dependent layer parts from the image
    void getLastVDLayerParts(std::vector<ViewDependentPart>& parts) const;

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            // number of rings -- number of filters: 2*((1-2^ringsNo)/(1-2))
            int ringsNo;
            // filter size in pixels
            int filterSize;
            // displaying variables and images for debugging
            int verbose;
            //scaling of raw int16 depth values into meters
            double scalingToMeters;
            //depth sensor model filenames
            std::string sensorFilename;
            // compute response if points no in filter window is bigger than threshold
            int backgroundThreshold;
    };

private:
    ///Configuration of the module
    Config config;
    ///Loaded filters
    hop3d::Filter::Seq filters;
    /// sensor model
    DepthSensorModel sensorModel;
    /// octets images
    std::vector<OctetsImage> octetsImages;
    /// parts images
    std::vector<PartsImage> partsImages;

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
    void normalizeVector(Vec3& normal) const;

    /// compute coordinate system from normal vector
    Mat33 coordinateFromNormal(const Vec3& _normal);

    ///extract octets from response image
    void extractOctets(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, hop3d::Octet::Seq& octets);

    /// compute otet for given location on response image
    void computeOctet(const std::vector< std::vector<Response> >& responseImg,  const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u,  int v, Octet& octet) const;

    /// compute max response in vindow
    void findMaxResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const;

    /// Fill in octet
    void fillInOctet(const OctetsImage& octetsImage, const ViewDependentPart::Seq& dictionary, int u, int v, Octet& octet) const;

    //set relative position for octets
    void computeRelativePositions(Octet& octet) const;

    /// determine id of the part using dictionary
    int findId(const ViewDependentPart::Seq& dictionary, const Octet& octet) const;
};

}
#endif // NORMAL_IMAGE_FILTER_H_INCLUDED
