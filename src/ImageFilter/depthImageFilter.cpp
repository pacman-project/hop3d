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
    if (depthImage.depth() == CV_16U) std::cout << "CV_16U == unsigned short " << sizeof(unsigned short) << std::endl;

    cv::Mat depthImageDouble(depthImage.cols,depthImage.rows, cv::DataType<double>::type);
    depthImage.convertTo(depthImageDouble,CV_64F);
    depthImageDouble =  depthImageDouble/3000;

    hop3d::ImagesDisplay displayer;
    displayer.displayDepthImage(depthImageDouble);
    std::vector<cv::Mat> filterImages;

    for(int iterF = 0 ; iterF < filters.size(); iterF++)
    {
        cv::Mat loadedFilter;
        cv::Mat loadedMask;
        loadedFilter = filters[iterF].patch.clone();
        loadedMask = filters[iterF].mask.clone();
        cv::transpose(loadedFilter,loadedFilter);
        cv::transpose(loadedMask,loadedMask);
        int numActive = cv::countNonZero(loadedMask);
//        std::cout << "Filter " << ": "<< std::endl << loadedFilter << std::endl << std::endl;
//        std::cout << "Mask active "<< numActive  << " : "<< std::endl << loadedMask << std::endl << std::endl;
        cv::Mat filterImage(depthImage.rows-(2*offset),depthImage.cols-(2*offset), cv::DataType<double>::type,cv::Scalar(0));
        std::vector<double> sValues;
        for(int i = offset; i < depthImageDouble.rows-(2*offset);i++){
            for(int j = offset; j < depthImageDouble.cols-(2*offset);j++){
//        for(int i = 175; i < 195;i++){
//            for(int j = 175; j < 195;j++){
                cv::Mat responseTemp(filterSize,filterSize, cv::DataType<double>::type);
                cv::Mat imageRoi(filterSize,filterSize, cv::DataType<double>::type);
                cv::Mat subResult(filterSize,filterSize, cv::DataType<double>::type);
                cv::Rect regionOfInterest = cv::Rect(j-offset, i-offset,filterSize,filterSize);

                //have to use clone otherwise it will go along different blocks of memory after performing substract
                //simple ROI is just a change of header to the same file when = is used it will start to point to the data in this other Mat
                imageRoi = depthImageDouble(regionOfInterest).clone();
                double middleValue = imageRoi.at<double>(offset,offset);
//                std::cout << "middleValue: " << middleValue << std::endl;
                cv::subtract(imageRoi,cv::Scalar(middleValue),subResult);
//                std::cout << "imageRoi " << i << "; " << j << ": " << std::endl << imageRoi << std::endl << std::endl;
//                std::cout << "filters " << i << "; " << j << ": " << std::endl << filters[iterF].patch << std::endl << std::endl;
//                std::cout << "subResult " << i << "; " << j << ": " << std::endl << subResult << std::endl << std::endl;
                cv::subtract(subResult,loadedFilter,responseTemp);
                responseTemp = responseTemp.mul(loadedMask);
                responseTemp = cv::abs(responseTemp);
//                std::cout << "responseTemp " << i << "; " << j << ": " << std::endl << responseTemp << std::endl << std::endl;
                double s = cv::sum(responseTemp)[0];
                if (s > 0.8 || s < -0.1) s = 0;
                else s *=(10000/numActive);
                //std::cout << "Value of s: " << i << "; " << j << ": " << s << std::endl;

                filterImage.at<double>(i-offset,j-offset) = s;
                sValues.push_back(s);
//                displayer.displayDepthImage(depthImageDouble);
                imageRoi.release();
                subResult.release();
                responseTemp.release();
            }
        }
//        std::cout << "Min max sValues: " << *std::min_element(sValues.begin(),sValues.end()) << ";" << *std::max_element(sValues.begin(),sValues.end()) << std::endl;
//        std::cout << "sValues:" <<std::endl;
//        for(auto it = sValues.begin(); it != sValues.end(); ++it) std::cout << *it << "; ";
//        std::cout << std::endl;
        filterImages.push_back(filterImage.clone());
        loadedFilter.release();
        loadedMask.release();
        filterImage.release();
    }

//        std::cout << "Image:" << std::endl << filterImages[iterI] << std::endl << std::endl;
        //displayer.displayDepthImage(filterImages[iterI]);
        cv::Mat responseImage(depthImage.rows-(2*offset),depthImage.cols-(2*offset), cv::DataType<double>::type,cv::Scalar(0));
        cv::Mat idImage(depthImage.rows-(2*offset),depthImage.cols-(2*offset), cv::DataType<int>::type,cv::Scalar(0));
        for(int i = 0; i < responseImage.rows;i++){
            for(int j = 0; j < responseImage.cols;j++){
                std::vector<double> minResponses;
                for(int iterI = 0 ; iterI < filterImages.size(); iterI++){
                        minResponses.push_back(filterImages[iterI].at<double>(j,i));
                }
                auto it = std::min_element(minResponses.begin(),minResponses.end());
                responseImage.at<double>(j,i) = *it;
                idImage.at<int>(j,i) = it - minResponses.begin();
            }
         }
         displayer.displayDepthImage(responseImage);
         displayer.displayDepthImage(idImage);


}

/// compute set of octets from set of the depth image
void DepthImageFilter::getOctets(Octet::Seq& octets){

}

/// compute set of octets from set of the ids image
void DepthImageFilter::getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){

}

/// get filters
void DepthImageFilter::getFilters(Filter::Seq& filters) const{
    filters = this->filters;
}

void DepthImageFilter::setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName)
{
    hop3d::Reader reader;
    reader.readFilters(patchesFileName,normalsFileName,masksFileName,filters);
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(void) {
    filter.reset(new DepthImageFilter());
    return filter.get();
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(std::string config) {
    filter.reset(new DepthImageFilter(config));
    return filter.get();
}
