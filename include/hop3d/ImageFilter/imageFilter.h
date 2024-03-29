/** @file imageFilter.h
 *
 * Image filter interface
 *
 */

#ifndef _IMAGEFILTER_H_
#define _IMAGEFILTER_H_

#include "../Data/Defs.h"
#include "hop3d/Data/Vocabulary.h"
#include "hop3d/Data/Graph.h"

namespace hop3d {

/// Image filterer interface
class ImageFilter {
public:

    /// Image filter type
    enum Type {
        /// depth image filter (patches)
        FILTER_DEPTH,
        /// depth image filter (normals)
        FILTER_NORMAL,
    };

    /// overloaded constructor
    ImageFilter(const std::string _name, Type _type) :
            name(_name), type(_type) {
    }

    /// Name of the filterer
    virtual const std::string& getName() const = 0;

    /// compute set of octets from set of the depth images
    virtual void computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets, bool inference) = 0;

    /// compute set of octets from set of the ids image
    virtual void getOctets(int layerNo, int categoryNo, int objectNo, int imageNo, Octet::Seq& octets, bool inference) = 0;

    /// get filters
    virtual void getFilters(Filter::Seq& _filters) const = 0;

    /// set filters
    virtual void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName) = 0;

    /// define 2rd layer octet images using selected words from third layer
    //virtual void computeImagesLastLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layersNo) = 0;
    /// define ith layer octet images using selected words from i+1 layer
    virtual void computePartsImage(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference) = 0;

    /// get last view dependent layer parts from the image
    virtual void getLayerParts(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference) const = 0;

    /// get set of ids for the given input point
    virtual void getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference) = 0;

    /// get set of ids for the given input point
    virtual void getRealisationsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference) = 0;

    /// returs filter ids and their position on the image
    virtual void getResponseFilters(int overlapNo, int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords, bool inference) const = 0;

    /// returs parts ids and their position on the image
    virtual void getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const = 0;

    /// returs parts ids and their position on the image
    virtual void getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoordsEucl>& partCoords, bool inference) const = 0;

    /// get cloud from dataset
    virtual void getCloud(const cv::Mat& depthImage, hop3d::PointCloudUV& cloud) const = 0;

    /// returs parts ids and their position on the image
    virtual void getPartsRealisation(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference) = 0;

    /// get input point
    //virtual void getPoint(int categoryNo, int objectNo, int imageNo, int u, int v, hop3d::Vec3& point) const = 0;

    /// get numbers of realisations
    virtual int getRealisationsNo(void) const = 0;

    /// define 2rd layer octet images using selected words from third layer
    virtual void identifyParts(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference, double distThreshold, std::vector<ViewDependentPart>& oldParts, std::vector<ViewDependentPart>& newParts) = 0;

    /// Virtual descrutor
    virtual ~ImageFilter() {
    }

protected:
    /// Filterer name
    const std::string name;

    /// Filterer type
    Type type;
};
}

#endif // _IMAGEFILTER_H_
