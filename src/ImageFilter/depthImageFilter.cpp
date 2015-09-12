#include "../include/ImageFilter/depthImageFilter.h"

using namespace hop3d;

/// A single instance of Features Map
DepthImageFilter::Ptr filter;

DepthImageFilter::DepthImageFilter(void) : ImageFilter("Depth image filter", FILTER_DEPTH) {
}

/// Construction
DepthImageFilter::DepthImageFilter(std::string config) :
        ImageFilter("Depth image filter", FILTER_DEPTH), config(config) {
}

/// Destruction
DepthImageFilter::~DepthImageFilter(void) {
}

///config class constructor
DepthImageFilter::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load depth filter config file.\n";
    tinyxml2::XMLElement * model = config.FirstChildElement( "Filterer" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("filtersNo", &filtersNo);
    std::cout << "Load filter parameters...\n";
    std::cout << "Filters no.: " << filtersNo << "\n";
}

const std::string& DepthImageFilter::getName() const {
    return name;
}

///compute set of octets from set of the depth images
void DepthImageFilter::computeOctets(const cv::Mat& depthImage, hop3d::Octet::Seq& octets){
    depthImage.type();//!!
    octets.size();//!!
    int filterSize = filters[0].patch.cols;
    int offset = int(filterSize/2); //casting discards fractional part
    std::cout << "Offset of filter " << offset << std::endl;
    if (depthImage.depth() == CV_16U) std::cout << "CV_16U == unsigned short " << sizeof(unsigned short) << std::endl;

    cv::Mat filterImage(depthImage.cols-(2*offset),depthImage.rows-(2*offset), cv::DataType<double>::type);
    for(int i = offset; i < depthImage.cols-(offset+1);i++){
        for(int j = offset; j < depthImage.rows-(offset+1);j++){
            cv::Rect regionOfInterest = cv::Rect(i-offset, j-offset,filterSize,filterSize);
            cv::Mat imageRoi = depthImage(regionOfInterest);
            //std::cout << imageRoi.at<unsigned short>(offset,offset) << std::endl;
            cv::Mat windowTemp(filterSize,filterSize, cv::DataType<double>::type);
            cv::Mat responseTemp(filterSize,filterSize, cv::DataType<double>::type);
            imageRoi.convertTo(windowTemp,CV_64F);
            windowTemp = windowTemp/5000;
            cv::subtract(windowTemp,windowTemp.at<double>(offset,offset),windowTemp);
            cv::subtract(windowTemp,filters[0].patch,responseTemp);
            double s = cv::sum(responseTemp)[0];
            filterImage.at<double>(i-offset,j-offset) = s;
        }
    }



}

/// get filters
void DepthImageFilter::getFilters(Filter::Seq& filters) const{
    filters = this->filters;
}

void DepthImageFilter::setFilters(std::string patchesFileName, std::string normalsFileName)
{
    hop3d::Reader reader;
    reader.readFilters(patchesFileName,normalsFileName,filters);
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(void) {
    filter.reset(new DepthImageFilter());
    return filter.get();
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(std::string config) {
    filter.reset(new DepthImageFilter(config));
    return filter.get();
}
