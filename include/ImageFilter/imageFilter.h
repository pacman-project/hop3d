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

/// Map interface
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

    /// Name of the map
    virtual const std::string& getName() const = 0;

    /// compute set of octets from set of the depth images
    virtual void computeOctets(const cv::Mat& depthImage, hop3d::Octet::Seq& octets) = 0;

    /// compute set of octets from set of the ids image
    virtual void getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets) = 0;

    /// get filters
    virtual void getFilters(Filter::Seq& _filters) const = 0;

    /// set filters
    virtual void setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName) = 0;

    /// define 2rd layer octet images using selected words from third layer
    virtual void computeImages3rdLayer(const ViewDependentPart::Seq& dictionary) = 0;

    /// get last view dependent layer parts from the image
    virtual void getLastVDLayerParts(std::vector<ViewDependentPart>& parts) const = 0;

    /// Virtual descrutor
    virtual ~ImageFilter() {
    }

protected:
    /// Map name
    const std::string name;

    /// Map type
    Type type;
};
}

#endif // _IMAGEFILTER_H_
