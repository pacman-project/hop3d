/** @file depthImageFilter.h
 *
 * implementation - depth image filter
 *
 */

#ifndef DEPTH_IMAGE_FILTER_H_INCLUDED
#define DEPTH_IMAGE_FILTER_H_INCLUDED

#include "imageFilter.h"
#include "hop3d/Utilities/Reader.h"
#include "hop3d/Utilities/Writer.h"

#include "hop3d/Utilities/ImagesDisplay.h"
#include <algorithm>    // std::min_element, std::max_element


#include "tinyXML/tinyxml2.h"
namespace hop3d {
    /// create a single depth image filter
    ImageFilter* createDepthImageFilter(void);
    /// create a single depth image filter
    ImageFilter* createDepthImageFilter(std::string config);


/// Map implementation
class DepthImageFilter: public ImageFilter {
public:
    /// Pointer
    typedef std::unique_ptr<DepthImageFilter> Ptr;

    /// Construction
    DepthImageFilter(void);

    /// Construction
    DepthImageFilter(std::string config);

    /// Destruction
    ~DepthImageFilter(void);

    /// Name of the map
    const std::string& getName() const;

    ///compute set of octets from set of the depth images
    void computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets, bool inference);

    /// compute set of octets from set of the depth image
    void getOctets(Octet::Seq& octets);

    /// compute set of octets from set of the ids image
    void getOctets(int layerNo, int categoryNo, int objectNo, int imageNo, Octet::Seq& octets, bool inference);

    /// get filters
    void getFilters(Filter::Seq& _filters) const;

    void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName);

    /// define 2rd layer octet images using selected words from third layer
    //void computeImagesLastLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layersNo);
    /// define ith layer octet images using selected words from i+1 layer
    void computePartsImage(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference);

    /// get last view dependent layer parts from the image
    void getLayerParts(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference) const;

    /// get set of ids for the given input point
    void getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference);

    /// get set of ids for the given input point
    void getRealisationsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference);

    /// returs filter ids and their position on the image
    void getResponseFilters(int overlapNo, int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords, bool inference) const;

    /// returs parts ids and their position on the image
    void getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const;

    /// returs parts ids and their position on the image
    void getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoordsEucl>& partCoords, bool inference) const;

    /// get cloud from dataset
    void getCloud(const cv::Mat& depthImage, hop3d::PointCloudUV& cloud) const;

    /// returs parts ids and their position on the image
    void getPartsRealisation(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference);

    /// get input point
    //void getPoint(int categoryNo, int objectNo, int imageNo, int u, int v, hop3d::Vec3& point) const;

    /// get numbers of realisations
    int getRealisationsNo(void) const;

    /// define 2rd layer octet images using selected words from third layer
    void identifyParts(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference, double distThreshold, std::vector<ViewDependentPart>& oldParts, std::vector<ViewDependentPart>& newParts);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            // number of filters
            int filtersNo;
            // filter size in pixels
            int filterSize;
            // displaying variables and images for debugging
            bool verbose;
            //overlap of octet Receptive Fields
            int overlapRf;
            //maximum value of depth for the sensor when not hitting the object
            int maxDepthValue;
            //scaling of raw int16 depth values into meters
            double scalingToMeters;
            //how many pixels in filtered window belong to object to treat it as a background
            int backgroundOverlap;
            //Response level on the edges which qualify it to an edge
            double boundaryResponseLevel;
            //Sum of the data in a patch which surpassed makes a background
            double backgroundValue;

    };

private:
    ///Configuration of the module
    Config config;
    ///Loaded filters
    hop3d::Filter::Seq filters;
    int filterSingleImageSingleFilter(const cv::Mat& depthImage, hop3d::Filter& filter, cv::Mat& filteredImage);
    int nonMaximaSuppression(const std::vector<cv::Mat>, cv::Mat& maxResponsesImage, cv::Mat& maxResponsesIdsImage);
    hop3d::Writer writer;
};

}
#endif // DEPTH_IMAGE_FILTER_H_INCLUDED
