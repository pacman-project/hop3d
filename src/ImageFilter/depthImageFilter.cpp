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
    octets.clear();

    int filterSize = filters[0].patch.cols;
    int overlapRf = 1; //overlap of the receptive fields in octets
    int offset = int(filterSize/2); //casting discards fractional part

//filters.size()

    hop3d::ImagesDisplay displayer;
    std::vector<cv::Mat> filteredImages(filters.size());
    //applying all the filters to an image
    unsigned long int e1 = cv::getTickCount();

//#pragma omp parallel for
    for(unsigned int iterF = 0 ; iterF < filters.size(); iterF++)
    {
        cv::Mat filteredImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<double>::type,cv::Scalar(0));
        filterSingleImageSingleFilter(depthImage,filters[iterF],filteredImage);

        //filteredImages.push_back(filteredImage.clone());
        filteredImages[iterF] = filteredImage.clone();
        filteredImage.release();

    }
    unsigned long int e2 = cv::getTickCount();
    double time = double((e2 - e1))/double(cv::getTickFrequency());
    std::cout << "Execution time: " << time << std::endl;

//Non-maxima suppression
        //displayer.displayDepthImage(filterImages[iterI]);
        cv::Mat responseImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<double>::type,cv::Scalar(0));
        cv::Mat idImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<int>::type,cv::Scalar(0));

        nonMaximaSuppression(filteredImages,responseImage,idImage);
        // displaying the results
        displayer.displayDepthImage(responseImage);
        displayer.displayDepthImage(idImage);

//filling in octets
         int octetSize = (filterSize-overlapRf)*2+filterSize;
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
                for(int k = offset; k < imageRoi.rows-(offset);k+=(filterSize-overlapRf)){
                    for(int l = offset; l < imageRoi.cols-(offset);l+=(filterSize-overlapRf)){
                        cv::Mat octetRoi(filterSize,filterSize, cv::DataType<double>::type);
                        cv::Mat octetIdRoi(filterSize,filterSize, cv::DataType<int>::type);
                        cv::Rect subRegion = cv::Rect(l-offset, k-offset,filterSize,filterSize);
                        octetRoi = imageRoi(subRegion).clone();
                        octetIdRoi = idRoi(subRegion).clone();
                        double minVal;
                        double maxVal;
                        cv::Point minLoc;
                        cv::Point maxLoc;
                        //location of the maximum point in the subelement of the octet
                        cv::minMaxLoc( octetRoi, &minVal, &maxVal, &minLoc, &maxLoc );
                        //searching within ROI
                        octetTemp.filterIds[octetIter/3][octetIter%3] = octetIdRoi.at<int>(maxLoc.x,maxLoc.y);
                        octetTemp.responses[octetIter/3][octetIter%3] = octetRoi.at<double>(maxLoc.x,maxLoc.y);
                        //searching within image
                        maxLoc.x +=(l+(j-octetOffset)+offset);
                        maxLoc.y +=(k+(i-octetOffset)+offset);
                        double  depthPoint = (double(depthImage.at<unsigned short>(maxLoc))/3000);
                        octetTemp.filterPos[octetIter/3][octetIter%3] = hop3d::ImageCoordsDepth(maxLoc.x,maxLoc.y,depthPoint);
                        octetIter++;
                        octetRoi.release();
                        octetIdRoi.release();
                    }
                }
//                //has to be generalized to larger neighbourhood
                double uVal = octetTemp.filterPos[1][1].u;
                double vVal = octetTemp.filterPos[1][1].v;
                double depthVal = octetTemp.filterPos[1][1].depth;

                for(int posIter = 0; posIter < octetIter; posIter++){
                    if (posIter == 4) continue;
                    octetTemp.filterPos[posIter/3][posIter%3].u -= uVal;
                    octetTemp.filterPos[posIter/3][posIter%3].v -= vVal;
                    octetTemp.filterPos[posIter/3][posIter%3].depth -= depthVal;
                }
              octets.push_back(octetTemp);
              imageRoi.release();
              idRoi.release();
             }
         }





}

/// compute set of octets from set of the depth image
void DepthImageFilter::getOctets(Octet::Seq& octets){
    std::cout << octets.size();
}

/// compute set of octets from set of the ids image
void DepthImageFilter::getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){
    std::cout << dictionary.size();
    std::cout << octets.size();
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

    //std::cout << "Applying filter number: " << filter.id << std::endl;
    int filterSize = filter.patch.cols; //size of the applied filter
    int offset = int(filterSize/2);     //casting discards fractional part
    //if (depthImage.depth() == CV_16U) std::cout << "CV_16U == unsigned short " << sizeof(unsigned short) << std::endl;
    //depth image represented as double giving it geometrical relation
    cv::Mat depthImageDouble(depthImage.cols,depthImage.rows, cv::DataType<double>::type);
    depthImage.convertTo(depthImageDouble,CV_64F);
    depthImageDouble =  depthImageDouble/3000;

    cv::Mat loadedFilter;
    cv::Mat loadedMask;
    loadedFilter = filter.patch.clone();
    filter.mask.convertTo(loadedMask,CV_8U);
    cv::transpose(loadedFilter,loadedFilter);
    cv::transpose(loadedMask,loadedMask);
    int numActive = cv::countNonZero(loadedMask);
    //looping over whole image
    cv::Mat imageRoi(filterSize,filterSize, cv::DataType<double>::type);
    cv::Mat responseTemp(filterSize,filterSize, cv::DataType<double>::type);
    cv::Mat subResult(filterSize,filterSize, cv::DataType<double>::type);
    //first approach with TBB, then GPU
    for(int i = offset; i < depthImageDouble.rows-(offset);i++){
        for(int j = offset; j < depthImageDouble.cols-(offset);j++){

            cv::Rect regionOfInterest = cv::Rect(j-offset, i-offset,filterSize,filterSize);
            imageRoi = depthImageDouble(regionOfInterest);
            double middleValue = imageRoi.at<double>(offset,offset);
            cv::subtract(imageRoi,cv::Scalar(middleValue),subResult,loadedMask);
            cv::absdiff(subResult,loadedFilter,responseTemp);
            double s = cv::sum(responseTemp)[0];
            if (s > 0.8) s = 0;
            //relation to number of active elements in the filter + scaling to get rid of problems with small floating point values
            else s *=(10000/numActive);
            filteredImage.at<double>(i-offset,j-offset) = s;
        }
    }
    imageRoi.release();
    subResult.release();
    responseTemp.release();
    loadedFilter.release();
    loadedMask.release();
    return 0;

}

int DepthImageFilter::nonMaximaSuppression(const std::vector<cv::Mat> responsesImages, cv::Mat &maxResponsesImage, cv::Mat &maxResponsesIdsImage)
{
    cv::Mat maxResponsesImageTemp(responsesImages[0].rows,responsesImages[0].cols, cv::DataType<double>::type,cv::Scalar(0));
    cv::Mat maxResponsesIdsImageTemp(responsesImages[0].rows,responsesImages[0].cols, cv::DataType<int>::type,cv::Scalar(0));
    std::vector<double> maxResponses;
    for(int i = 0; i < maxResponsesImageTemp.rows;i++){
        for(int j = 0; j < maxResponsesImageTemp.cols;j++){
            std::vector<double> minResponses;
            for(unsigned int iterI = 0 ; iterI < responsesImages.size(); iterI++){
                    minResponses.push_back(responsesImages[iterI].at<double>(j,i));
            }
            auto itMin = std::min_element(minResponses.begin(),minResponses.end());
            auto itMax = std::max_element(minResponses.begin(),minResponses.end());
            maxResponses.push_back(*itMax);
            maxResponsesImageTemp.at<double>(j,i) = *itMin;
            maxResponsesIdsImageTemp.at<int>(j,i) = int(itMin - minResponses.begin());
        }

     }
     auto max = std::max_element(maxResponses.begin(),maxResponses.end());
     double maxValue = *max;
     maxResponsesImageTemp = maxResponsesImageTemp.mul(cv::Scalar(-1));
     maxResponsesImageTemp = maxResponsesImageTemp + maxValue;
     maxResponsesImageTemp = maxResponsesImageTemp.mul(cv::Scalar(1/maxValue));
     maxResponsesImage = maxResponsesImageTemp.clone();
     maxResponsesIdsImage = maxResponsesIdsImageTemp.clone();
     maxResponsesImageTemp.release();
     maxResponsesIdsImageTemp.release();
     return 0;
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(void) {
    filter.reset(new DepthImageFilter());
    return filter.get();
}

hop3d::ImageFilter* hop3d::createDepthImageFilter(std::string config) {
    filter.reset(new DepthImageFilter(config));
    return filter.get();
}
