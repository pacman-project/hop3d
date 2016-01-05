#include "hop3d/ImageFilter/depthImageFilter.h"

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
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load depth filter config file: " + filename);
    tinyxml2::XMLElement * model = config.FirstChildElement( "Filterer" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("filtersNo", &filtersNo);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("filterSize", &filterSize);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("overlapRf", &overlapRf);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("maxDepthValue", &maxDepthValue);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("scalingToMeters", &scalingToMeters);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("backgroundOverlap", &backgroundOverlap);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("boundaryResponseLevel", &boundaryResponseLevel);

    backgroundValue = double(maxDepthValue/scalingToMeters)*0.97;

    std::cout << "Load filter parameters...\n";
    std::cout << "Filters no.: " << filtersNo << "\n";
    std::cout<< "Filter size in pixels: " << filterSize << std::endl;
    std::cout<< "Verbose: " << verbose << std::endl;
    std::cout<< "Overlap of octets receptive fields: " << overlapRf << std::endl;
    std::cout<< "Maximum value of depth for the sensor when not hitting the object: " << maxDepthValue << std::endl;
    std::cout<< "Scaling of raw int16 depth values into meters: " << scalingToMeters << std::endl;
    std::cout<< "How many pixels in filtered patch belong to object to treat it as a background: " << backgroundOverlap << std::endl;
    std::cout<< "Response level on the edges which qualify it to an edge: " << boundaryResponseLevel << std::endl;
    std::cout<< "Sum of the data in a patch which surpassed makes a background: " << backgroundValue << std::endl;

 }

const std::string& DepthImageFilter::getName() const {
    return name;
}

///compute set of octets from set of the depth images
void DepthImageFilter::computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets){
    std::cout << "Unused parameters: " << categoryNo << " " << objectNo << " " << imageNo << "\n";
    octets.clear();

    int filterSize = filters[0].patch.cols;
    int offset = int(filterSize/2); //casting discards fractional part

//filters.size()

    hop3d::ImagesDisplay displayer;
    std::vector<cv::Mat> filteredImages(filters.size());
    //applying all the filters to an image
    unsigned long int e1 = (unsigned long)cv::getTickCount();

//#pragma omp parallel for
    for(unsigned int iterF = 0 ; iterF < filters.size(); iterF++)
    {
        cv::Mat filteredImage(depthImage.rows-(offset),depthImage.cols-(offset), cv::DataType<double>::type,cv::Scalar(-1));
        filterSingleImageSingleFilter(depthImage,filters[iterF],filteredImage);
        filteredImages[iterF] = filteredImage.clone();
        filteredImage.release();

    }
    unsigned long int e2 = (unsigned long) cv::getTickCount();
    double time = double((e2 - e1))/double(cv::getTickFrequency());
    std::cout << "Execution time: " << time << std::endl;

//Non-maxima suppression
        //displayer.displayDepthImage(filterImages[iterI]);
        cv::Mat responseImage(depthImage.cols-(offset),depthImage.rows-(offset), cv::DataType<double>::type,cv::Scalar(0));
        cv::Mat idImage(depthImage.cols-(offset),depthImage.rows-(offset), cv::DataType<int>::type,cv::Scalar(0));

        nonMaximaSuppression(filteredImages,responseImage,idImage);
        // displaying the results
        if(config.verbose){
            displayer.displayDepthImage(responseImage);
            displayer.displayDepthImage(idImage);
        }

//filling in octets
         int octetSize = (filterSize-config.overlapRf)*2+filterSize;
         int octetOffset = int(octetSize/2);
         for(int i = octetOffset; i < responseImage.rows-(octetOffset);i+=(octetSize)){
            for(int j = octetOffset; j < responseImage.cols-(octetOffset);j+=(octetSize)){
                cv::Mat imageRoi(octetSize,octetSize, cv::DataType<double>::type);
                cv::Mat idRoi(octetSize,octetSize, cv::DataType<int>::type);
                cv::Rect regionOfInterest = cv::Rect(j-octetOffset, i-octetOffset,octetSize,octetSize);
                //have to use clone otherwise it will go along different blocks of memory after performing substract
                //simple ROI is just a change of header to the same file when = is used it will start to point to the data in this other Mat
                imageRoi = responseImage(regionOfInterest).clone();
                idRoi = idImage(regionOfInterest).clone();
                hop3d::Octet octetTemp;
                int octetIter = 0;
                for(int k = offset; k < imageRoi.rows-(offset);k+=(filterSize-config.overlapRf)){
                    for(int l = offset; l < imageRoi.cols-(offset);l+=(filterSize-config.overlapRf)){
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
                        octetTemp.partIds[octetIter/3][octetIter%3] = octetIdRoi.at<int>(maxLoc.x,maxLoc.y);
                        octetTemp.responses[octetIter/3][octetIter%3] = octetRoi.at<double>(maxLoc.x,maxLoc.y);
                        //searching within image
                        maxLoc.x +=(l+(j-octetOffset)+offset);
                        maxLoc.y +=(k+(i-octetOffset)+offset);
                        double  depthPoint = (double(depthImage.at<unsigned short>(maxLoc))/config.scalingToMeters);
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


//                if(config.verbose){
//                    for(int posIter = 0; posIter < octetIter; posIter++){
//                        if (posIter == 4) continue;
//                        std::cout << "(u,v,d) neigh. [" << (posIter/3)-1  << ", " << (posIter%3)-1 << "]: ("<< octetTemp.filterPos[posIter/3][posIter%3].u << ", "<< octetTemp.filterPos[posIter/3][posIter%3].v << ", "<< octetTemp.filterPos[posIter/3][posIter%3].depth << ")  ";
//                        std::cout << "(id,activ.) neigh. [" << (posIter/3)-1  << ", " << (posIter%3)-1 << "]: ("<< octetTemp.filterIds[posIter/3][posIter%3] << ", "<< octetTemp.responses [posIter/3][posIter%3] << ")  ";
//                    }
//                    std::cout << std::endl << "(u,v,d) central: ("<< octetTemp.filterPos[1][1].u << ", "<< octetTemp.filterPos[1][1].v << ", "<< octetTemp.filterPos[1][1].depth << ")" << std::endl;
//                }

                for (int n=0;n<3;n++)
                    for (int m=0;m<3;m++){
                        if (octetTemp.partIds[n][m]==-1){
                            octetTemp.filterPos[n][m].depth=0;
                        }
                    }
              if (octetTemp.filterPos[1][1].depth <  21 && octetTemp.partIds[0][0]!=-1 && octetTemp.partIds[0][1]!=-1 && octetTemp.partIds[0][2]!=-1 && octetTemp.partIds[1][0]!=-1 && octetTemp.partIds[1][1]!=-1 && octetTemp.partIds[1][2]!=-1 && octetTemp.partIds[2][0]!=-1 && octetTemp.partIds[2][1]!=-1 && octetTemp.partIds[2][2]!=-1){
                  if(config.verbose){
                      for(int posIter = 0; posIter < octetIter; posIter++){
                          if (posIter == 4) continue;
                          std::cout << "neigh. [" << (posIter/3)-1  << ", " << (posIter%3)-1 << "] (u,v,d): ("<< octetTemp.filterPos[posIter/3][posIter%3].u << ", "<< octetTemp.filterPos[posIter/3][posIter%3].v << ", "<< octetTemp.filterPos[posIter/3][posIter%3].depth << ")  ";
                          std::cout << " (id,activ.): ("<< octetTemp.partIds[posIter/3][posIter%3] << ", "<< octetTemp.responses [posIter/3][posIter%3] << ")  ";
                      }
                      std::cout << std::endl << "(u,v,d) central: ("<< octetTemp.filterPos[1][1].u << ", "<< octetTemp.filterPos[1][1].v << ", "<< octetTemp.filterPos[1][1].depth << ")" << std::endl;
                  }
                   octets.push_back(octetTemp);
            }

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
void DepthImageFilter::getOctets(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){
    std::cout << categoryNo << " " << objectNo << " " << imageNo << "\n";
    std::cout << dictionary.size();
    std::cout << octets.size();
}

/// get filters
void DepthImageFilter::getFilters(Filter::Seq& _filters) const{
    _filters = this->filters;
}

void DepthImageFilter::setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName)
{
    hop3d::Reader reader;
    reader.readFilters(patchesFileName,normalsFileName,masksFileName,filters);
}

/// define 2rd layer octet images using selected words from third layer
/*void DepthImageFilter::computeImagesLastLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layersNo){
    std::cout << categoryNo << " " << objectNo << " " << imageNo << " " << layersNo << "\n";
    dictionary.size();
}*/

/// define ith layer octet images using selected words from i+1 layer
void DepthImageFilter::computePartsImage(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layerNo){
    std::cout << categoryNo << " " << objectNo << " " << imageNo << " " << layerNo << "\n";
    dictionary.size();
}

/// get last view dependent layer parts from the image
void DepthImageFilter::getLayerParts(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts) const{
    std::cout << categoryNo << " " << objectNo << " " << imageNo << layerNo << "\n";
    parts.clear();
}

/// get set of ids for the given input point
void DepthImageFilter::getPartsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart, int layersNo){
    std::cout << "not implemented\n";
    ids.clear();
    std::cout << lastVDpart.id << "\n";
    std::cout << categoryNo << " " << objectNo << " " << imageNo << " " << u << " " << v  << layersNo << "\n";
}

/// returs filter ids and their position on the image
void DepthImageFilter::getResponseFilters(int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords) const{
    std::cout << categoryNo << objectNo << imageNo << "\n";
    partCoords.clear();
}

/// returs parts ids and their position on the image
void DepthImageFilter::getParts3D(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const{
    std::cout << categoryNo << objectNo << imageNo << layerNo << "\n";
    partCoords.clear();
}

/// get cloud from dataset
void DepthImageFilter::getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud) const{
    cloud.clear();
    std::cout << categoryNo << " " << objectNo << imageNo << "\n";
}

int DepthImageFilter::filterSingleImageSingleFilter(const cv::Mat &depthImage, Filter &filter, cv::Mat &filteredImage)
{

    //std::cout << "Applying filter number: " << filter.id << std::endl;
    int filterSize = filter.patch.cols; //size of the applied filter
    int offset = int(filterSize/2);     //casting discards fractional part
    //if (depthImage.depth() == CV_16U) std::cout << "CV_16U == unsigned short " << sizeof(unsigned short) << std::endl;
    //depth image represented as double giving it geometrical relation
    cv::Mat depthImageDouble(depthImage.rows,depthImage.cols, cv::DataType<double>::type);
    depthImage.convertTo(depthImageDouble,CV_64F);
    depthImageDouble =  depthImageDouble/config.scalingToMeters;

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
    for(int i = offset; i < depthImageDouble.cols-(offset);i++){
        for(int j = offset; j < depthImageDouble.rows-(offset);j++){
                cv::Rect regionOfInterest = cv::Rect(i-offset, j-offset,filterSize,filterSize);
                imageRoi = depthImageDouble(regionOfInterest);
                if(!imageRoi.isContinuous()) imageRoi = imageRoi.clone();
                double middleValue = imageRoi.at<double>(offset,offset);
                cv::subtract(imageRoi,cv::Scalar(middleValue),subResult,loadedMask);
                if(middleValue > config.backgroundValue) continue;
                cv::absdiff(subResult,loadedFilter,responseTemp);
                double s = cv::sum(responseTemp)[0];
                if (s > config.boundaryResponseLevel) s = -2;
                //relation to number of active elements in the filter + scaling to get rid of problems with small floating point values
                else s *=(10000/numActive);
                filteredImage.at<double>(j-offset,i-offset) = s;
        }
    }
  //  writer.matrixToTxtFile(filteredImage,"filteredImage.txt");
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
    for(int i = 0; i < maxResponsesImageTemp.cols;i++){
        for(int j = 0; j < maxResponsesImageTemp.rows;j++){
            std::vector<double> minResponses;
            if(responsesImages[0].at<double>(j,i) < 0 ) {
                maxResponses.push_back(0);
                maxResponsesImageTemp.at<double>(j,i) = 0;
                maxResponsesIdsImageTemp.at<int>(j,i) = -1;

            }
            else{
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
