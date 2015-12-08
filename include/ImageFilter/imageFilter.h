/** @file imageFilter.h
 *
 * Image filter interface
 *
 */

#ifndef _IMAGEFILTER_H_
#define _IMAGEFILTER_H_

#include "../Data/Defs.h"
#include "Data/Vocabulary.h"

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
    virtual void computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets) = 0;

    /// compute set of octets from set of the ids image
    virtual void getOctets(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, Octet::Seq& octets) = 0;

    /// get filters
    virtual void getFilters(Filter::Seq& _filters) const = 0;

    /// set filters
    virtual void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName) = 0;

    /// define 2rd layer octet images using selected words from third layer
    virtual void computeImages3rdLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary) = 0;

    /// get last view dependent layer parts from the image
    virtual void getLastVDLayerParts(int categoryNo, int objectNo, int imageNo, std::vector<ViewDependentPart>& parts) const = 0;

    /// get set of ids for the given input point
    virtual void getPartsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart) = 0;

    /// returs filter ids and their position on the image
    virtual void getResponseFilters(int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords) const = 0;

    /// returs parts ids and their position on the image
    virtual void getParts3D(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const = 0;

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
