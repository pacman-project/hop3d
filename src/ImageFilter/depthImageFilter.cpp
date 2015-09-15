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
    int overlapRf = 1; //overlap of the receptive fields in octets
    int offset = int(filterSize/2); //casting discards fractional part
    if (depthImage.depth() == CV_16U) std::cout << "CV_16U == unsigned short " << sizeof(unsigned short) << std::endl;

    cv::Mat depthImageDouble(depthImage.cols,depthImage.rows, cv::DataType<double>::type);
    depthImage.convertTo(depthImageDouble,CV_64F);
    depthImageDouble =  depthImageDouble/3000;

    hop3d::ImagesDisplay displayer;
    displayer.displayDepthImage(depthImageDouble);
    std::vector<cv::Mat> filterImages;
    //applying all the filters to an image
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
        //boundary of the image is not taken into account
        cv::Mat filterImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<double>::type,cv::Scalar(0));
        std::vector<double> sValues;
        //looping over whole image
        for(int i = offset; i < depthImageDouble.rows-(offset);i++){
            for(int j = offset; j < depthImageDouble.cols-(offset);j++){
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
                //condition not to take into account boundaries of the object
                if (s > 0.8 || s < -0.1) s = 0;
                //relation to number of active elements in the filter + scaling to get rid of problems with small floating point values
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
        cv::Mat responseImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<double>::type,cv::Scalar(0));
        cv::Mat idImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<int>::type,cv::Scalar(0));
        std::vector<double> maxResponses;
        for(int i = 0; i < responseImage.rows;i++){
            for(int j = 0; j < responseImage.cols;j++){
                std::vector<double> minResponses;
                for(int iterI = 0 ; iterI < filterImages.size(); iterI++){
                        minResponses.push_back(filterImages[iterI].at<double>(j,i));
                }
                auto itMin = std::min_element(minResponses.begin(),minResponses.end());
                auto itMax = std::max_element(minResponses.begin(),minResponses.end());
                maxResponses.push_back(*itMax);
                responseImage.at<double>(j,i) = *itMin;
                idImage.at<int>(j,i) = itMin - minResponses.begin();
            }

         }
         auto max = std::max_element(maxResponses.begin(),maxResponses.end());
         double maxValue = *max;
         responseImage = responseImage.mul(cv::Scalar(-1));
         responseImage = responseImage + maxValue;
         responseImage = responseImage.mul(cv::Scalar(1/maxValue));
         displayer.displayDepthImage(responseImage);
         displayer.displayDepthImage(idImage);


         int octetSize = (filterSize-overlapRf)*3;
         int octetOffset = int(octetSize/2);
         for(int i = octetOffset; i < responseImage.rows-(octetOffset-2*overlapRf);i+=(octetSize-overlapRf)){
            for(int j = octetOffset; j < responseImage.cols-(octetOffset-2*overlapRf);j+=(octetSize-overlapRf)){
                cv::Mat imageRoi(octetSize,octetSize, cv::DataType<double>::type);
                cv::Mat idRoi(octetSize,octetSize, cv::DataType<int>::type);
                cv::Rect regionOfInterest = cv::Rect(j-octetOffset, i-octetOffset,octetSize,octetSize);
                //have to use clone otherwise it will go along different blocks of memory after performing substract
                //simple ROI is just a change of header to the same file when = is used it will start to point to the data in this other Mat
                imageRoi = responseImage(regionOfInterest).clone();
                idRoi = idImage(regionOfInterest).clone();
                hop3d::Octet octetTemp;
                int octetIter = 0;
//                for(int k = offset; k < imageRoi.rows-(offset);k+=(offset-overlapRf)){
//                    for(int l = offset; l < imageRoi.cols-(offset);l+=(offset-overlapRf)){
//                        cv::Mat octetRoi(filterSize,filterSize, cv::DataType<double>::type);
//                        cv::Mat octetIdRoi(filterSize,filterSize, cv::DataType<int>::type);
//                        cv::Rect subRegion = cv::Rect(l-offset, k-offset,filterSize,filterSize);
//                        octetRoi = imageRoi(subRegion).clone();
//                        octetIdRoi = idRoi(subRegion).clone();
//                        double minVal;
//                        double maxVal;
//                        cv::Point minLoc;
//                        cv::Point maxLoc;
//                        //location of the maximum point in the subelement of the octet
//                        cv::minMaxLoc( octetRoi, &minVal, &maxVal, &minLoc, &maxLoc );
//                        maxLoc.x +=(l+j+octetOffset+offset);
//                        maxLoc.y +=(k+i+octetOffset+offset);
//                        double depthPoint;
//                        depthPoint = depthImageDouble.at<double>(maxLoc);
//                        octetTemp.filterPos[octetIter/3][octetIter%3] = hop3d::ImageCoordsDepth(maxLoc.x,maxLoc.y,depthPoint);
//                        octetRoi.release();
//                    }
//                }

              imageRoi.release();
              idRoi.release();
             }
         }



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

int DepthImageFilter::filterSingleImageSingleFilter(const cv::Mat &depthImage, Filter &filter, cv::Mat &filteredImage)
{

}

hop3d::ImageFilter* hop3d::createDepthImageFilter(void) {
    filter.reset(new DepthImageFilter());
    return filter.get();
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(std::string config) {
    filter.reset(new DepthImageFilter(config));
    return filter.get();
}
