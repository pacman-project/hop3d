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

    /// compute set of octets from set of the depth image
    void getOctets(Octet::Seq& octets);

    /// compute set of octets from set of the ids image
    void getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets);

    /// get filters
    void getFilters(Filter::Seq& _filters) const;

    void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName);
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
            bool verbose;
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
};

}
#endif // NORMAL_IMAGE_FILTER_H_INCLUDED
