#include "hop3d/Utilities/ImagesDisplay.h"

hop3d::ImagesDisplay::ImagesDisplay(){

}

hop3d::ImagesDisplay::~ImagesDisplay(){

}



int hop3d::ImagesDisplay::displayDepthImage(const cv::Mat &inputImage)
{
    double min;
    double max;
    cv::minMaxIdx(inputImage, &min, &max);
    std::cout << "Max min depht image: " << min << "; " << max << std::endl;
    cv::Mat changeInputImage;
    changeInputImage = inputImage.clone();
    changeInputImage = changeInputImage - min;
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    changeInputImage.convertTo(adjMap,CV_8U, 255/(max-min), 0);

    // this is great. It converts your grayscale image into a tone-mapped one,
    // much more pleasing for the eye
    // function is found in contrib module, so include contrib.hpp
    // and link accordingly
    cv::Mat falseColorsMap;
/*    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
    //cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
    cv::namedWindow("Depth image", CV_WINDOW_AUTOSIZE );
    cv::imshow("Depth image", falseColorsMap);
    cv::waitKey(0);*/
    adjMap.release();
    falseColorsMap.release();
    return 0;
}

int hop3d::ImagesDisplay::displayDepthImage(const cv::Mat &inputImage, double min, double max)
{
    std::cout << "Max min depht image: " << min << "; " << max << std::endl;
    cv::Mat changeInputImage;
    changeInputImage = inputImage.clone();
    changeInputImage = changeInputImage - min;
    cv::Mat adjMap;
    // expand your range to 0..255. Similar to histEq();
    changeInputImage.convertTo(adjMap,CV_8U, 255/(max-min), 0);

    // this is great. It converts your grayscale image into a tone-mapped one,
    // much more pleasing for the eye
    // function is found in contrib module, so include contrib.hpp
    // and link accordingly
    cv::Mat falseColorsMap;
/*    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
    //cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);
    cv::namedWindow("Depth image", CV_WINDOW_AUTOSIZE );
    cv::imshow("Depth image", falseColorsMap);
    cv::waitKey(0);*/
    adjMap.release();
    falseColorsMap.release();
    return 0;
}
