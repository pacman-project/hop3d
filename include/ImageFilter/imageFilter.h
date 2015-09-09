/** @file imageFilter.h
 *
 * Image filter interface
 *
 */

#ifndef _IMAGEFILTER_H_
#define _IMAGEFILTER_H_

#include "../Data/Defs.h"

namespace hop3d {

/// Map interface
class ImageFilter {
public:

    /// Image filter type
    enum Type {
        /// depth image filter
        FILTER_DEPTH,
    };

    /// overloaded constructor
    ImageFilter(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the map
    virtual const std::string& getName() const = 0;

    ///compute set of octets from set of the depth images
    virtual void computeOctets(const cv::Mat& depthImage, std::vector<Octet>& octets) = 0;

    /// Virtual descrutor
    virtual ~ImageFilter() {
    }

protected:
    /// Map name
    const std::string name;

    /// Map type
    Type type;
};
};

#endif // _IMAGEFILTER_H_
