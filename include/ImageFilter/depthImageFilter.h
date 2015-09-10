/** @file depthImageFilter.h
 *
 * implementation - depth image filter
 *
 */

#ifndef DEPTH_IMAGE_FILTER_H_INCLUDED
#define DEPTH_IMAGE_FILTER_H_INCLUDED

#include "imageFilter.h"
#include "../../external/tinyXML/tinyxml2.h"

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
    void computeOctets(const cv::Mat& depthImage, std::vector<Octet>& octets);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            // number of filters
            int filtersNo;
    };

private:
    ///Configuration of the module
    Config config;
};

}
#endif // DEPTH_IMAGE_FILTER_H_INCLUDED
