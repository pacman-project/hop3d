#ifndef UTILITIES_IMAGESDISPLAY_H
#define UTILITIES_IMAGESDISPLAY_H

#include "Data/Defs.h"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

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
