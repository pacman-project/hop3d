#ifndef UTILITIES_IMAGESDISPLAY_H
#define UTILITIES_IMAGESDISPLAY_H

#include "hop3d/Data/Defs.h"

namespace hop3d {
class ImagesDisplay
{
public:
    ImagesDisplay();
    ~ImagesDisplay();
    int displayDepthImage(const cv::Mat &inputImage);
    int displayDepthImage(const cv::Mat &inputImage, double min, double max);

};

}
#endif // UTILITIES_IMAGESDISPLAY_H
