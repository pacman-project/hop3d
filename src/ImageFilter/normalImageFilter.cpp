#include "hop3d/ImageFilter/normalImageFilter.h"
#include <chrono>

using namespace hop3d;

/// A single instance of Features Map
NormalImageFilter::Ptr filterNorm;

NormalImageFilter::NormalImageFilter(void) : ImageFilter("Depth image filter using normals", FILTER_NORMAL) {
    generateFilters();
    partRealisationsCounter = 0;
}

/// Construction
NormalImageFilter::NormalImageFilter(std::string config, std::string sensorConfig) :
        ImageFilter("Depth image filter using normals", FILTER_NORMAL), config(config), sensorModel(sensorConfig) {
    generateFilters();
    partRealisationsCounter = 0;
}

/// Destruction
NormalImageFilter::~NormalImageFilter(void) {
}

///config class constructor
NormalImageFilter::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load depth filter (normal) config file: " + filename);
    tinyxml2::XMLElement * model = config.FirstChildElement( "NormalFilterer" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("ringsNo", &ringsNo);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("filterSize", &filterSize);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("nonMaximumSupressionGroup", &nonMaximumSupressionGroup);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minOctetSize", &minOctetSize);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minOctetSizeSecondLayer", &minOctetSizeSecondLayer);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minPointsNoSecondLayer", &minPointsNoSecondLayer);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("distThresholdSecondLayer", &distThresholdSecondLayer);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("distThresholdThirdLayer", &distThresholdThirdLayer);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("useEuclideanCoordinates", &useEuclideanCoordinates);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("splitSurfaces", &splitSurfaces);

    model->FirstChildElement( "PCA" )->QueryIntAttribute("PCAWindowSize", &PCAWindowSize);
    model->FirstChildElement( "PCA" )->QueryDoubleAttribute("PCADistThreshold", &PCADistThreshold);
    model->FirstChildElement( "PCA" )->QueryDoubleAttribute("PCARelDistClusters", &PCARelDistClusters);
    model->FirstChildElement( "PCA" )->QueryBoolAttribute("PCAuseClustering", &PCAuseClustering);
    model->FirstChildElement( "PCA" )->QueryIntAttribute("PCAbackgroundThreshold", &PCAbackgroundThreshold);

    model->FirstChildElement( "imageFiltering" )->QueryBoolAttribute("useMedianFilter", &useMedianFilter);
    model->FirstChildElement( "imageFiltering" )->QueryIntAttribute("kernelSize", &kernelSize);
    model->FirstChildElement( "imageFiltering" )->QueryBoolAttribute("useMeanFilter", &useMeanFilter);
    model->FirstChildElement( "imageFiltering" )->QueryIntAttribute("meanKernelSize", &meanKernelSize);

    if (verbose>0){
        std::cout << "Load normal filter parameters...\n";
        std::cout << "Rings no.: " << ringsNo << "\n";
        std::cout<< "Filter size in pixels: " << filterSize << std::endl;
        std::cout<< "Verbose: " << verbose << std::endl;
        //std::cout<< "Scaling of raw int16 depth values into meters: " << scalingToMeters << std::endl;
    }

 }

const std::string& NormalImageFilter::getName() const {
    return name;
}

/// get cloud from dataset
void NormalImageFilter::getCloud(const cv::Mat& depthImage, hop3d::PointCloudUV& cloud) const{
    cloud.clear();
    double scale = 1/sensorModel.config.depthImageScale;
    cv::Mat filteredImg = depthImage.clone();
    filterDepthImage(depthImage, filteredImg);
    std::vector< std::vector<hop3d::PointNormal> > cloudOrd(depthImage.rows, std::vector<hop3d::PointNormal> (depthImage.cols));
    std::vector< std::vector<hop3d::PointNormal> > cloudOrdNorm(depthImage.rows, std::vector<hop3d::PointNormal> (depthImage.cols));
    for (int i=0;i<filteredImg.rows;i++){
        for (int j=0;j<filteredImg.cols;j++){
            PointNormalUV puv;
            sensorModel.getPoint(j, i, filteredImg.at<uint16_t>(i,j)*scale, cloudOrd[i][j].position);
            sensorModel.getPoint(i, j, filteredImg.at<uint16_t>(i,j)*scale, cloudOrdNorm[i][j].position);
            sensorModel.getPoint(j, i, filteredImg.at<uint16_t>(i,j)*scale, puv.position);
            if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                puv.u=i; puv.v=j;
                cloud.push_back(puv);
            }
        }
    }
    int pointIdx=0;
    for (int i=config.filterSize/2;i<filteredImg.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<filteredImg.cols-(config.filterSize/2);j++){
            if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                computeNormal(i, j, cloudOrdNorm);
                cloud[pointIdx].normal = cloudOrdNorm[i][j].normal;
                pointIdx++;
            }
        }
    }
}

/// get point from dataset
/*void NormalImageFilter::getPoint(int categoryNo, int objectNo, int imageNo, int u, int v, hop3d::Vec3& point) const{
  //  std::cout << "search for " << u << ", " << v << "\n";
    for (auto &pointUV : inputClouds[categoryNo][objectNo][imageNo]){
//        std::cout << "uv " << pointUV.u << ", " << pointUV.v << "\n";
        if ((pointUV.u==(unsigned int)u)&&(pointUV.v==(unsigned int)v)){
    //        getchar();
            sensorModel.getPoint(v,u,pointUV.position(2),point);
            //std::cout << "found " << point.transpose() << "\n";
            return;
        }
    }
    point = Vec3(NAN,NAN,NAN);
}*/

/// compute median
uint16_t NormalImageFilter::median(const cv::Mat& inputImg, int u, int v, int kernelSize) const{
    int size = kernelSize/2;
    std::vector<short unsigned int> values;
    for (int i=v-size;i<v+size+1;i++){
        for (int j=u-size;j<u+size+1;j++){
            if ((i>=0)&&(j>=0)&&(j<inputImg.rows)&&(i<inputImg.cols)){
                values.push_back(inputImg.at<uint16_t>(j,i));
            }
        }
    }
    std::sort(values.begin(), values.end());
    if (values.size()>0){
        return (values[values.size()/2]);
    }
    else
        return 0;
}

///Apply median filter on the image
void NormalImageFilter::medianFilter(const cv::Mat& inputImg, cv::Mat& outputImg, int kernelSize) const{
	for (int i = 0; i<inputImg.rows; i++){
        for (int j=0;j<inputImg.cols;j++){
            outputImg.at<uint16_t>(i,j)=median(inputImg, i, j, kernelSize);
        }
    }
}

/// filter depth image
void NormalImageFilter::filterDepthImage(const cv::Mat& input, cv::Mat& output) const{
    if (config.useMedianFilter){
        if (config.kernelSize<6)
            cv::medianBlur(input, output, config.kernelSize);
        else {
            medianFilter(input, output, config.kernelSize);
        }
    }
    if (config.useMeanFilter){
        cv::GaussianBlur(input, output, cv::Size(config.meanKernelSize,config.meanKernelSize),0,0);
    }
}

///compute set of octets from set of the depth images
void NormalImageFilter::computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets, bool inference){
    octets.clear();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    /// compute point cloud, keep order
    std::vector< std::vector<hop3d::PointNormal> > cloudOrd(depthImage.rows, std::vector<hop3d::PointNormal> (depthImage.cols));
    //PointCloudUV cloudGrasp;
    /// response: id, response value
    std::vector< std::vector<Response> > responseImage(depthImage.rows, std::vector<Response>(depthImage.cols,std::make_pair<int, double>(-1,-1.0)));
    double scale = 1/sensorModel.config.depthImageScale;
    if (config.verbose>1)
        imshow( "Input image", depthImage );
    cv::Mat filteredImg = depthImage.clone();
    filterDepthImage(depthImage, filteredImg);
    if (config.verbose>1)
        imshow( "Filtered image", filteredImg );
    for (int i=0;i<filteredImg.rows;i++){
        for (int j=0;j<filteredImg.cols;j++){
            sensorModel.getPoint(i, j, filteredImg.at<uint16_t>(i,j)*scale, cloudOrd[i][j].position);
        }
    }
    int pointIdx = 0;
    cv::Mat idsImage(filteredImg.rows,filteredImg.cols, cv::DataType<int>::type,cv::Scalar(0));
    for (int i=config.filterSize/2;i<filteredImg.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<filteredImg.cols-(config.filterSize/2);j++){
            if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                computeNormal(i, j, cloudOrd);
				if (!std::isnan(double(cloudOrd[i][j].normal(2)))){
                    responseImage[i][j].first = toId(cloudOrd[i][j].normal);
                    //compute response (dot product)
                    responseImage[i][j].second = cloudOrd[i][j].normal.adjoint()*filters[responseImage[i][j].first].normal;
                    if (config.verbose==2)
                        idsImage.at<uchar>(i,j)=(uchar)(toId(cloudOrd[i][j].normal)*2);
                }
                pointIdx++;
            }
        }
    }
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        Octet::Seq octetsTmp;
        OctetsImage octetsImage = extractOctets(responseImage, overlapNo, cloudOrd, octetsTmp);
        octets.insert(octets.end(), octetsTmp.begin(), octetsTmp.end() );
        if (inference)
            updateOctetsImage(0,overlapNo,categoryNo, objectNo, imageNo, octetsImagesInference, octetsImage);
        else
            updateOctetsImage(0,overlapNo,categoryNo, objectNo, imageNo, octetsImages, octetsImage);
    }

    if (config.verbose>0){
        std::chrono::steady_clock::time_point end=std::chrono::steady_clock::now();
        std::cout << "Octets extraction takes " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    }
    if (config.verbose==2){
        namedWindow( "Ids image", cv::WINDOW_AUTOSIZE );// Create a window for display.
        imshow( "Ids image", idsImage );
        cv::waitKey(0);
        getchar();
    }
}

/// update structure which holds octets images
void NormalImageFilter::updateOctetsImage(int layerNo, int overlapNo, int categoryNo, int objectNo, int imageNo, OctetsImages& _octetsImages, const OctetsImage& octetsImage){
    if((int)_octetsImages.size()<=layerNo){
        _octetsImages.resize(layerNo+1);
    }
    if((int)_octetsImages[layerNo].size()<=overlapNo){
        _octetsImages[layerNo].resize(overlapNo+1);
    }
    if((int)_octetsImages[layerNo][overlapNo].size()<=categoryNo){
        _octetsImages[layerNo][overlapNo].resize(categoryNo+1);
    }
    if ((int)_octetsImages[layerNo][overlapNo][categoryNo].size()<=objectNo){
        _octetsImages[layerNo][overlapNo][categoryNo].resize(objectNo+1);
    }
    if ((int)_octetsImages[layerNo][overlapNo][categoryNo][objectNo].size()<=imageNo){
        _octetsImages[layerNo][overlapNo][categoryNo][objectNo].resize(imageNo+1);
    }
    _octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo] = octetsImage;
}

/// update structure which holds octets images
/*void NormalImageFilter::saveCloud(int categoryNo, int objectNo, int imageNo, const hop3d::PointCloudUV& cloud){
    if((int)inputClouds.size()<=categoryNo){
        inputClouds.resize(categoryNo+1);
    }
    if ((int)inputClouds[categoryNo].size()<=objectNo){
        inputClouds[categoryNo].resize(objectNo+1);
    }
    if ((int)inputClouds[categoryNo][objectNo].size()<=imageNo){
        inputClouds[categoryNo][objectNo].resize(imageNo+1);
    }
    inputClouds[categoryNo][objectNo][imageNo] = cloud;
}*/

/// returs filter ids and their position on the image
void NormalImageFilter::getResponseFilters(int overlapNo, int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords, bool inference) const {
    partCoords.clear();
    const OctetsImages* oImgs;
    if (inference)
        oImgs = &octetsImagesInference;
    else
        oImgs = &octetsImages;
    for (auto& row : (*oImgs)[0][overlapNo][categoryNo][objectNo][imageNo]){
        for (auto& octet : row){
            if (octet.get()!=nullptr){
                if (!octet->isBackground){
                    for (int i=0;i<3;i++){
                        for (int j=0;j<3;j++){
                            if (octet->partIds[i][j]>=0){
                                if (i==1&&j==1){
                                    PartCoords fcoords(octet->partIds[i][j], octet->filterPos[i][j], octet->offsets[i][j]);
                                    partCoords.push_back(fcoords);
                                    //if (overlapNo==0&&categoryNo==0&&objectNo==0&&imageNo==0)
                                        //octet->print();
                                        //std::cout << "part coords1 " << fcoords.coords << " \n" << fcoords.offset.matrix() << "\n";
                                }
                                else{
                                    PartCoords fcoords(octet->partIds[i][j], octet->filterPos[1][1]+octet->filterPos[i][j], octet->offsets[i][j]);
                                    partCoords.push_back(fcoords);
                                    //if (overlapNo==0&&categoryNo==0&&objectNo==0&&imageNo==0)
                                        //octet->print();
                                        //std::cout << "part coords " << fcoords.coords << " \n" << fcoords.offset.matrix() << "\n";
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/// returs parts ids and their position on the image
void NormalImageFilter::getPartsRealisation(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference){
    OctetsImages* oImages;
    PartsImages* pImages;
    if (inference){
        oImages = &octetsImagesInference;
        pImages = &partsImagesInference;
    }
    else {
        oImages = &octetsImages;
        pImages = &partsImages;
    }
    parts.clear();
    if (layerNo==0){
        for (size_t row=0; row<(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo].size();row++){
            for (size_t col=0; col<(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row].size();col++){
                if ((*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col].get()!=nullptr){
                    if (!(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->isBackground){
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                if ((*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->partIds[i][j]>=0){
                                    ViewDependentPart vdp;
                                    if (i==1&&j==1)
                                        vdp.locationEucl = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->partsPosNorm[1][1].mean.block<3,1>(0,0);
                                    else
                                        vdp.locationEucl = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->partsPosNorm[1][1].mean.block<3,1>(0,0)+(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->partsPosNorm[i][j].mean.block<3,1>(0,0);
                                    Mat34 offset(Mat34::Identity());
                                    offset.matrix().block<3,3>(0,0) = coordinateFromNormal((*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->partsPosNorm[i][j].mean.block<3,1>(3,0));
                                    vdp.offset=offset;
                                    vdp.id = 0;
                                    vdp.realisationId = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->realisationsIds[i][j];
                                    parts.push_back(vdp);
                                }
                            }
                        }
                    }
                    if ((*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet.size()>0){
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                if (octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet[0].partIds[i][j]>=0){
                                    ViewDependentPart vdp;
                                    if (i==1&&j==1)
                                        vdp.locationEucl = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet[0].partsPosEucl[1][1];
                                    else
                                        vdp.locationEucl = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet[0].partsPosEucl[1][1]+(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet[0].partsPosEucl[i][j];
                                    vdp.offset=(*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->secondOctet[0].offsets[i][j];
                                    vdp.id = 0;
                                    vdp.realisationId = (*oImages)[layerNo][overlapNo][categoryNo][objectNo][imageNo][row][col]->realisationsIds[i][j];
                                    parts.push_back(vdp);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else{
        for (auto& row : (*pImages)[layerNo-1][overlapNo][categoryNo][objectNo][imageNo]){
            for (auto& part : row){
                if (part.get()!=nullptr){
                    if (!part->isBackground()){
                        parts.push_back(*part);
                        if (part->secondVDPart.size()>0)
                            parts.push_back(part->secondVDPart[0]);
                    }
                }
            }
        }
    }
}

/// returs parts ids and their position on the image
void NormalImageFilter::getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const{
    partCoords.clear();
    if (layerNo==1){
        for (auto& row : octetsImages[1][overlapNo][categoryNo][objectNo][imageNo]){
            for (auto& octet : row){
                if (octet.get()!=nullptr){
                    if (!octet->isBackground){
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                if (octet->partIds[i][j]>=0){
                                    if (i==1&&j==1){
                                        PartCoords fcoords(octet->partIds[i][j], octet->filterPos[i][j], octet->offsets[i][j]);
                                        partCoords.push_back(fcoords);
                                    }
                                    else{
                                        PartCoords fcoords(octet->partIds[i][j], octet->filterPos[1][1]+octet->filterPos[i][j], octet->offsets[i][j]);
                                        partCoords.push_back(fcoords);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else{
        for (auto& row : partsImages[layerNo-1][overlapNo][categoryNo][objectNo][imageNo]){
            for (auto& part : row){
                if (part.get()!=nullptr){
                    if (!part->isBackground()){
                        PartCoords fcoords(part->id, part->location, part->offset);
                        partCoords.push_back(fcoords);
                    }
                }
            }
        }
    }
}

/// returs parts ids and their position on the image
void NormalImageFilter::getParts3D(int overlapNo, int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoordsEucl>& partCoords, bool inference) const{
    partCoords.clear();
    const PartsImages* pImgs;
    if (inference)
        pImgs = &partsImagesInference;
    else
        pImgs = &partsImages;
    if (layerNo>0){
        for (auto& row : (*pImgs)[layerNo-1][overlapNo][categoryNo][objectNo][imageNo]){
            for (auto& part : row){
                if (part.get()!=nullptr){
                    if (!part->isBackground()){
                        PartCoordsEucl fcoords(part->id, part->locationEucl, part->offset);
                        partCoords.push_back(fcoords);
                        if (part->secondVDPart.size()>0){
                            PartCoordsEucl fcoords1(part->secondVDPart[0].id, part->secondVDPart[0].locationEucl, part->secondVDPart[0].offset);
                            partCoords.push_back(fcoords1);
                        }
                    }
                }
            }
        }
    }
}

/// update structure which holds parts images
void NormalImageFilter::updatePartsImages(int categoryNo, int objectNo, int imageNo, int layerNo, int overlapNo, PartsImages& _partsImages, const PartsImage& partsImage){
    if((int)_partsImages.size()<=layerNo){
        _partsImages.resize(layerNo+1);
    }
    if((int)_partsImages[layerNo].size()<=overlapNo){
        _partsImages[layerNo].resize(overlapNo+1);
    }
    if((int)_partsImages[layerNo][overlapNo].size()<=categoryNo){
        _partsImages[layerNo][overlapNo].resize(categoryNo+1);
    }
    if ((int)_partsImages[layerNo][overlapNo][categoryNo].size()<=objectNo){
        _partsImages[layerNo][overlapNo][categoryNo].resize(objectNo+1);
    }
    if ((int)_partsImages[layerNo][overlapNo][categoryNo][objectNo].size()<=imageNo){
        _partsImages[layerNo][overlapNo][categoryNo][objectNo].resize(imageNo+1);
    }
    _partsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo] = partsImage;
}

///extract octets from response image
NormalImageFilter::OctetsImage NormalImageFilter::extractOctets(const std::vector< std::vector<Response> >& responseImg, int overlapNo, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, hop3d::Octet::Seq& octets){
    int u=0;
    OctetsImage octetsImage(responseImg.size()/(config.filterSize*3), std::vector<std::shared_ptr<Octet>> (responseImg[0].size()/(config.filterSize*3)));
    for (size_t i=((overlapNo+1)*config.filterSize)+config.filterSize/2;i<responseImg.size()-config.filterSize-(config.filterSize/2);i=i+3*config.filterSize){
        int v=0;
        for (size_t j=((overlapNo+1)*config.filterSize)+config.filterSize/2;j<responseImg[0].size()-config.filterSize-(config.filterSize/2);j=j+3*config.filterSize){
            Octet octet;
            if (computeOctet(responseImg, cloudOrd, int(i), int(j), octet)){
                updateRealisationIds(octet);
                octetsImage[u][v].reset(new Octet(octet));
                octets.push_back(octet);
                if (octet.secondOctet.size()>0)
                    octets.push_back(octet.secondOctet[0]);
                /*octet.print();
                getchar();*/
            }
            v++;
        }
        u++;
    }
    return octetsImage;
}

/// update realisations ids
void NormalImageFilter::updateRealisationIds(Octet& octet){
    for (int k=0;k<(int)octet.realisationsIds.size();k++){//update realisations ids
        for (int l=0;l<(int)octet.realisationsIds.size();l++){
            if (octet.partIds[k][l]>=0){
                octet.realisationsIds[k][l]=partRealisationsCounter;
                partRealisationsCounter++;
            }
        }
    }
    if (octet.secondOctet.size()>0){
        for (int k=0;k<(int)octet.secondOctet[0].realisationsIds.size();k++){//update realisations ids
            for (int l=0;l<(int)octet.secondOctet[0].realisationsIds.size();l++){
                if (octet.secondOctet[0].partIds[k][l]>=0){
                    octet.secondOctet[0].realisationsIds[k][l]=partRealisationsCounter;
                    partRealisationsCounter++;
                }
            }
        }
    }
}

/// compute otet for given location on response image
bool NormalImageFilter::computeOctet(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u,  int v, Octet& octet) const{
    int elementsNo=0;
    for (int i=-1;i<2;i++){//for neighbouring blocks in octet
        for (int j=-1;j<2;j++){
            if (config.nonMaximumSupressionGroup){
                if (findMaxGroupResponse(responseImg, cloudOrd, u+i*config.filterSize, v+j*config.filterSize, octet, i+1, j+1))
                    elementsNo++;
            }
            else{
                if (findMaxResponse(responseImg, cloudOrd, u+i*config.filterSize, v+j*config.filterSize, octet, i+1, j+1))
                    elementsNo++;
            }
            //findMeanResponse(cloudOrd, u+i*config.filterSize, v+j*config.filterSize, octet, i+1, j+1);
        }
    }
    if (elementsNo>config.minOctetSize){//set relative position for octets
        int biggerGroupSize, smallerGroupSize;
        bool hasDoubleSurface = octet.hasDoubleSurface(config.PCADistThreshold,biggerGroupSize, smallerGroupSize);
        if (hasDoubleSurface && biggerGroupSize<config.minOctetSize)
            return false;
        if (octet.partIds[1][1]<0){
            octet.filterPos[1][1].u = v;
            octet.filterPos[1][1].v = u;
            if (config.useEuclideanCoordinates){
                sensorModel.getPoint(u,v,1.0,octet.partsPosEucl[1][1]);
            }
        }
        computeRelativePositions(octet, 1);
        octet.isBackground=false;
        if (hasDoubleSurface&&config.splitSurfaces){
            octet.splitSurfaces(config.PCADistThreshold, config.minOctetSize, smallerGroupSize);
        }
        return true;
    }
    return false;
}

//set relative position for octets
void NormalImageFilter::computeRelativePositions(Octet& octet, int layerNo) const{
    double meanDepth=0;
    Vec6 meanPosNorm(Vec6::Zero());
    double min=std::numeric_limits<double>::max();
    double max=std::numeric_limits<double>::min();
    //double globU=0, globV=0;
    if (octet.partIds[1][1]<0){
        int depthNo=0;
        ViewDependentPart part;
        part.partIds = octet.partIds;
        part.partsPosNorm = octet.partsPosNorm;
        if (layerNo==1)
            ViewDependentPart::removeSecondSurface(part,config.PCADistThreshold);
        else if (layerNo==2)
            ViewDependentPart::removeSecondSurface(part,config.distThresholdSecondLayer);
        else if (layerNo==3){
            ViewDependentPart::removeSecondSurface(part,config.distThresholdSecondLayer);
        }
        else{
            throw std::runtime_error("Distance threshold for layer " + std::to_string(layerNo) + " undefined\n");
        }
        Octet octetTmp(octet);
        octetTmp.partIds=part.partIds;
        for (int i=0;i<3;i++){//compute mean depth
            for (int j=0;j<3;j++){
                if (octetTmp.partIds[i][j]>=0){
                    meanDepth+= octetTmp.filterPos[i][j].depth;
                    meanPosNorm+=octetTmp.partsPosNorm[i][j].mean;
                    if (min>octetTmp.filterPos[i][j].depth){
                        min = octetTmp.filterPos[i][j].depth;
                        //globU = octet.filterPos[i][j].u;
                        //globV = octet.filterPos[i][j].v;
                    }
                    if (max<octetTmp.filterPos[i][j].depth){
                        max = octetTmp.filterPos[i][j].depth;
                    }
                    depthNo++;
                }
            }
        }
        meanDepth /= double(depthNo);
        meanPosNorm /= double(depthNo);
        Vec3 normMean(meanPosNorm(3), meanPosNorm(4), meanPosNorm(5));
        normMean.normalize();
        meanPosNorm(3) = normMean(0); meanPosNorm(4) = normMean(1); meanPosNorm(5) = normMean(2);
        octet.filterPos[1][1].depth = meanDepth;
        octet.partsPosNorm[1][1].mean = meanPosNorm;
        if (config.useEuclideanCoordinates)
            octet.partsPosEucl[1][1] = meanPosNorm.block<3,1>(0,0);
        //octet.filterPos[1][1].depth = min;
        //octet.filterPos[1][1].u = globU;
        //octet.filterPos[1][1].v = globV;
    }
    //sensorModel.getPoint(octet.filterPos[1][1].u,octet.filterPos[1][1].v,octet.filterPos[1][1].depth,octet.partsPosEucl[1][1]);
    for (int i=0;i<3;i++){//for neighbouring blocks
        for (int j=0;j<3;j++){
            if (!((i==1)&&(j==1))){
                if (octet.partIds[i][j]<0){
                    octet.filterPos[i][j].u=(i-1)*config.filterSize*(2.0*layerNo-1.0);//(j-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].v=(j-1)*config.filterSize*(2.0*layerNo-1.0);//(i-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].depth=meanDepth;
                    if (config.useEuclideanCoordinates){
                        octet.partsPosEucl[i][j]-=octet.partsPosEucl[1][1];
                        //sensorModel.getPoint(octet.filterPos[i][j].u,octet.filterPos[i][j].v,octet.filterPos[i][j].depth,octet.partsPosEucl[i][j]);
                    }
                    //octet.partsPosNorm[i][j].mean.block<3,1>(0,0)-=octet.partsPosNorm[1][1].mean.block<3,1>(0,0);//should be boxplus
                    octet.partsPosNorm[i][j].mean(0)=octet.filterPos[i][j].v*0.001;//should be boxplus
                    octet.partsPosNorm[i][j].mean(1)=octet.filterPos[i][j].u*0.001;
                    octet.partsPosNorm[i][j].mean(2)=0;
                }
                else {
                    if (config.useEuclideanCoordinates){
                        //sensorModel.getPoint(octet.filterPos[i][j].u,octet.filterPos[i][j].v,octet.filterPos[i][j].depth,octet.partsPosEucl[i][j]);
                        octet.partsPosEucl[i][j]-=octet.partsPosEucl[1][1];
                    }
                    octet.filterPos[i][j].u-=octet.filterPos[1][1].u;
                    octet.filterPos[i][j].v-=octet.filterPos[1][1].v;
                    octet.filterPos[i][j].depth-=octet.filterPos[1][1].depth;
                    octet.partsPosNorm[i][j].mean.block<3,1>(0,0)-=octet.partsPosNorm[1][1].mean.block<3,1>(0,0);//should be boxplus
                }
            }
        }
    }
}

/// compute max response in the window
bool NormalImageFilter::computeNormalStats(const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Vec6& mean, Mat66& cov) const{
    mean = Vec6::Zero();
    bool isBackground=true;
    int normalsNo=0;
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            if ((!std::isnan(double(cloudOrd[u+i][v+j].normal(2))))&&(!std::isnan(double(cloudOrd[u+i][v+j].position(0))))){
                Vec6 posNorm;
                posNorm << cloudOrd[u+i][v+j].position, cloudOrd[u+i][v+j].normal;
                mean+=posNorm;
                isBackground = false;
                normalsNo++;
            }
        }
    }
    if (normalsNo<2)
        return false;
    mean*=(1.0/double(normalsNo));
    mean.block<3,1>(3,0).normalized();
    cov.setZero();
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            if ((!std::isnan(double(cloudOrd[u+i][v+j].normal(2))))&&(!std::isnan(double(cloudOrd[u+i][v+j].position(0))))){
                Vec6 posNorm;
                posNorm << cloudOrd[u+i][v+j].position, cloudOrd[u+i][v+j].normal;
                cov+=(posNorm-mean)*(posNorm-mean).transpose();
            }
        }
    }
    cov*=(1.0/double(normalsNo));
    return !isBackground;// succes if not background
}

/// compute mean response in the window
bool NormalImageFilter::findMeanResponse(const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const{
    return computeNormalStats(cloudOrd, u, v, octet.partsPosNorm[idx][idy].mean, octet.partsPosNorm[idx][idy].covariance);
}

/// compute max response in the window
bool NormalImageFilter::findMaxResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const{
    octet.responses[idx][idy]=-1;    octet.partIds[idx][idy]=-1;
    bool isBackground=true;
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            if (responseImg[u+i][v+j].second>octet.responses[idx][idy]){
                octet.responses[idx][idy] = responseImg[u+i][v+j].second;
                octet.partIds[idx][idy] = responseImg[u+i][v+j].first;
                ImageCoordsDepth coord;
                coord.u = v+j; coord.v = u+i; coord.depth = cloudOrd[u+i][v+j].position(2);
                octet.filterPos[idx][idy] = coord;
                if (config.useEuclideanCoordinates)
                    sensorModel.getPoint(coord.u, coord.v, coord.depth, octet.partsPosEucl[idx][idy]);
                isBackground = false;
            }
        }
    }
    return !isBackground;// succes if not background
}

/// compute max response for the most numerous group in the window
bool NormalImageFilter::findMaxGroupResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const{
    octet.responses[idx][idy]=-1;    octet.partIds[idx][idy]=-1;
    std::map<int,int> occurencesMap;//filter id and number of occurences
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            std::pair<std::map<int,int>::iterator,bool> ret;
            if (responseImg[u+i][v+j].first>=0){
                ret = occurencesMap.insert ( std::pair<int,int>(responseImg[u+i][v+j].first,1) );
                if (ret.second==false) {
                    occurencesMap[responseImg[u+i][v+j].first]=ret.first->second+1;
                }
            }
        }
    }
    bool isBackground=true;
    if (occurencesMap.size()>0){
        int maxCount=0;
        for (auto& occur : occurencesMap){// find max occurences
            if (occur.second>maxCount)
                maxCount = occur.second;
        }
        std::vector<int> maxIds;// most countable ids
        for (auto& occur : occurencesMap){
            if (occur.second==maxCount)
                maxIds.push_back(occur.first);
        }
        Vec3 meanPos(0,0,0);
        Vec3 meanNormal(0,0,0);
        int pointsNo=0;
        for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
            for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
                if ((responseImg[u+i][v+j].second>octet.responses[idx][idy])&&(std::find(maxIds.begin(), maxIds.end(), responseImg[u+i][v+j].first) != maxIds.end())){
                    octet.responses[idx][idy] = responseImg[u+i][v+j].second;
                    octet.partIds[idx][idy] = responseImg[u+i][v+j].first;
                    ImageCoordsDepth coord;
                    coord.u = v+j; coord.v = u+i; coord.depth = cloudOrd[u+i][v+j].position(2);
                    octet.filterPos[idx][idy] = coord;
                    //if (config.useEuclideanCoordinates)
                    sensorModel.getPoint(coord.u, coord.v, coord.depth, octet.partsPosEucl[idx][idy]);
                    meanPos+=octet.partsPosEucl[idx][idy];
                    meanNormal += cloudOrd[u+i][v+j].normal;
                    pointsNo++;
                    isBackground = false;
                }
            }
        }
        if (pointsNo>0){
            meanPos/=double(pointsNo);
            meanNormal/=double(pointsNo);
            meanNormal.normalize();
            octet.partsPosEucl[idx][idy]=meanPos;
            octet.partsPosNorm[idx][idy].mean.block<3,1>(0,0)=meanPos;
            octet.partsPosNorm[idx][idy].mean.block<3,1>(3,0)=meanNormal;
            //std::cout << "pos no " << pointsNo << " " << meanPos.transpose() << " " << meanNormal.transpose() << "\n";
            //getchar();
        }
    }
    return !isBackground;// succes if not background
}

/// compute set of octets from set of the ids image
void NormalImageFilter::getOctets(int layerNo, int categoryNo, int objectNo, int imageNo, Octet::Seq& octets, bool inference){
    octets.clear();
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        OctetsImage octetsImage;
        PartsImage partsImage;
        if (inference){
            octetsImage = octetsImagesInference[layerNo-1][overlapNo][categoryNo][objectNo][imageNo];
            partsImage = partsImagesInference[layerNo-1][overlapNo][categoryNo][objectNo][imageNo];
        }
        else{
            octetsImage = octetsImages[layerNo-1][overlapNo][categoryNo][objectNo][imageNo];
            partsImage = partsImages[layerNo-1][overlapNo][categoryNo][objectNo][imageNo];
        }
        OctetsImage nextLayerOctetsImg(octetsImage.size()/3, std::vector<std::shared_ptr<Octet>> (octetsImage.back().size()/3));
        int u=0;
        for (size_t i=1+overlapNo; i<octetsImage.size()-1;i=i+3){
            int v=0;
            for (size_t j=1+overlapNo; j<octetsImage[0].size()-1;j=j+3){
                Octet octet;
                if (!isBackground(octetsImage, (int)i, (int)j)){
                    if (fillInOctet(octetsImage, partsImage, (int)i, (int)j, octet)>=config.minOctetSizeSecondLayer){
                        int biggerGroupSize, smallerGroupSize;
                        bool hasDoubleSurface = octet.hasDoubleSurface(config.distThresholdSecondLayer,biggerGroupSize, smallerGroupSize);
                        if (biggerGroupSize>=config.minOctetSizeSecondLayer){
                            computeRelativePositions(octet, layerNo+1);
                            octet.isBackground=false;
                            if (hasDoubleSurface&&config.splitSurfaces){
                                octet.splitSurfaces(config.distThresholdSecondLayer, config.minOctetSizeSecondLayer, smallerGroupSize);
                            }
                            updateRealisationIds(octet);
                            octets.push_back(octet);
                            if (octet.secondOctet.size()>0)
                                octets.push_back(octet.secondOctet[0]);
                            nextLayerOctetsImg[u][v].reset(new Octet(octet));
                        }
                    }
                }
                v++;
            }
            u++;
        }
        if (inference)
            updateOctetsImage(layerNo, overlapNo, categoryNo, objectNo, imageNo, octetsImagesInference, nextLayerOctetsImg);
        else
            updateOctetsImage(layerNo, overlapNo, categoryNo, objectNo, imageNo, octetsImages, nextLayerOctetsImg);
    }
}

/// check if octet is background
bool NormalImageFilter::isBackground(OctetsImage& octetsImage, int u, int v) const{
    for (int i=-1;i<2;i++){
        for (int j=-1;j<2;j++){
            if (octetsImage[i+u][j+v].get()!=nullptr){
                if (!octetsImage[i+u][j+v]->isBackground){
                    return false;
                }
            }
        }
    }
    return true;
}

/// define 2rd layer octet images using selected words from third layer
/*void NormalImageFilter::computeImagesLastLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layersNo){
    OctetsImage octetsImage;
    if (layersNo==1)
        octetsImage = octetsImages1stLayer[categoryNo][objectNo][imageNo];
    else if (layersNo==2)
        octetsImage = octetsImages2ndLayer[categoryNo][objectNo][imageNo];
    PartsImage partsImage (octetsImage.size(), std::vector<ViewDependentPart> (octetsImage.back().size()));

//    PartsImage partsImage (octetsImages2ndLayer[categoryNo][objectNo][imageNo].size(), std::vector<ViewDependentPart> (octetsImages2ndLayer[categoryNo][objectNo][imageNo].back().size()));
  //  OctetsImage octetsImage = octetsImages2ndLayer[categoryNo][objectNo][imageNo];
    int partsNo = 0;
    for (size_t i=0; i<octetsImage.size();i++){
        for (size_t j=0; j<octetsImage.back().size();j++){
            ViewDependentPart part;
            part.id=-1;
            //std::cout << "(" << i << "," << j << ")" << octetsImage[i][j].filterIds[1][1] << ", ";
            //if (!isBackground(octetsImage,(int)i, (int)j)){
            if (!octetsImage[i][j].isBackground){
                //std::cout << dictionary.size() << "\n";
                //std::cout << "findId(dictionary,octetsImage[i][j]) " << findId(dictionary,octetsImage[i][j]) << "\n";
                Mat34 offset;
                int id = findId(dictionary,octetsImage[i][j], offset);
                part = dictionary[id];
                part.offset = offset;
                part.id = id;
                part.layerId=layersNo+1;
                //std::cout << "found id : " << findId(dictionary,octetsImage[i][j]) << " new id " << dictionary[findId(dictionary,octetsImage[i][j])].id << "\n";
                //std::cout << "parts no " << partsNo << "\n";
                part.location = octetsImage[i][j].filterPos[1][1];
                part.locationEucl = octetsImage[i][j].partsPosEucl[1][1];
                part.gaussians[1][1].mean=Vec3(octetsImage[i][j].filterPos[1][1].u, octetsImage[i][j].filterPos[1][1].v, octetsImage[i][j].filterPos[1][1].depth);
                partsNo++;
            }
            partsImage[i][j]=part;
        }
        //std::cout << "\n";
    }
    updatePartsImages(categoryNo, objectNo, imageNo, partsImage);
}*/

/// define 2rd layer octet images using selected words from third layer
void NormalImageFilter::computePartsImage(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference){
    //std::cout << "layer no , overlapNo, int categoryNo, int objectNo, int imageNo " << layerNo << ", " << overlapNo << ", " << categoryNo << ", " << objectNo << ", " << imageNo << "\n";
    //std::cout << "octetsImages.size " << octetsImages.size() << "\n";
    //std::cout << "octetsImages[layerNo].size " << octetsImages[layerNo].size() << "\n";
    //std::cout << "octetsImages[layerNo][overlapNo].size " << octetsImages[layerNo][overlapNo].size() << "\n";
    //std::cout << "octetsImages[layerNo][overlapNo][categoryNo].size " << octetsImages[layerNo][overlapNo][categoryNo].size() << "\n";
    //std::cout << "octetsImages[layerNo][overlapNo][categoryNo][objectNo].size " << octetsImages[layerNo][overlapNo][categoryNo][objectNo].size() << "\n";
    //std::cout << "octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo].size " << octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo].size() << "\n";
    OctetsImage octetsImage;
    if (inference)
        octetsImage = octetsImagesInference[layerNo][overlapNo][categoryNo][objectNo][imageNo];
    else
        octetsImage = octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo];
    PartsImage partsImage (octetsImage.size(), std::vector<std::shared_ptr<ViewDependentPart>> (octetsImage.back().size()));
    for (size_t i=0; i<octetsImage.size();i++){
        for (size_t j=0; j<octetsImage.back().size();j++){
            ViewDependentPart part;
            part.id=-1;
            if (octetsImage[i][j].get()!=nullptr){
                if (!octetsImage[i][j]->isBackground){
                    Mat34 offset;
                    double distance;
                    int id = findId(hierarchy, layerNo,*(octetsImage[i][j]), distance, offset);
                    part = hierarchy.viewDependentLayers[layerNo][id];
                    part.offset = offset;
                    part.id = id;
                    part.realisationId = partRealisationsCounter;
                    partRealisationsCounter++;
                    part.layerId=layerNo+2;
                    part.location = octetsImage[i][j]->filterPos[1][1];
                    part.locationEucl = octetsImage[i][j]->partsPosEucl[1][1];
                    part.group.clear();
                    //part.offsets = octetsImage[i][j].offsets;
                    //part.locationEucl = octetsImage[i][j]->partsPosNorm[1][1].mean.block<3,1>(0,0);
                    part.gaussians[1][1].mean=Vec3(octetsImage[i][j]->filterPos[1][1].u, octetsImage[i][j]->filterPos[1][1].v, octetsImage[i][j]->filterPos[1][1].depth);
                    if (octetsImage[i][j].get()->secondOctet.size()>0){
                        ViewDependentPart secondPart;
                        if (!octetsImage[i][j].get()->secondOctet[0].isBackground){
                            double distParts;
                            id = findId(hierarchy, layerNo, octetsImage[i][j].get()->secondOctet[0], distParts, offset);
                            secondPart = hierarchy.viewDependentLayers[layerNo][id];
                            secondPart.offset = offset;
                            secondPart.id = id;
                            secondPart.realisationId = partRealisationsCounter;
                            partRealisationsCounter++;
                            secondPart.layerId=layerNo+2;
                            secondPart.location = octetsImage[i][j].get()->secondOctet[0].filterPos[1][1];
                            secondPart.locationEucl = octetsImage[i][j].get()->secondOctet[0].partsPosEucl[1][1];
                            secondPart.gaussians[1][1].mean=Vec3(octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].u, octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].v, octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].depth);
                            secondPart.offsets = octetsImage[i][j].get()->secondOctet[0].offsets;
                            secondPart.group.clear();
                            part.secondVDPart.push_back(secondPart);
                        }
                    }
                    partsImage[i][j].reset(new ViewDependentPart(part));
                }
            }
        }
    }
    if (inference)
        updatePartsImages(categoryNo, objectNo, imageNo, layerNo, overlapNo, partsImagesInference, partsImage);
    else
        updatePartsImages(categoryNo, objectNo, imageNo, layerNo, overlapNo, partsImages, partsImage);
}

/// define 2rd layer octet images using selected words from third layer
void NormalImageFilter::identifyParts(int overlapNo, int categoryNo, int objectNo, int imageNo, const Hierarchy& hierarchy, int layerNo, bool inference, double distThreshold, std::vector<ViewDependentPart>& oldParts, std::vector<ViewDependentPart>& newParts){
    newParts.clear();
    oldParts.clear();
    OctetsImage octetsImage;
    if (inference)
        octetsImage = octetsImagesInference[layerNo][overlapNo][categoryNo][objectNo][imageNo];
    else
        octetsImage = octetsImages[layerNo][overlapNo][categoryNo][objectNo][imageNo];
    for (size_t i=0; i<octetsImage.size();i++){
        for (size_t j=0; j<octetsImage.back().size();j++){
            ViewDependentPart part;
            part.id=-1;
            if (octetsImage[i][j].get()!=nullptr){
                if (!octetsImage[i][j]->isBackground){
                    Mat34 offset;
                    double distance;
                    int id = findId(hierarchy, layerNo,*(octetsImage[i][j]), distance, offset);
                    part = hierarchy.viewDependentLayers[layerNo][id];
                    part.offset = offset;
                    part.realisationId = partRealisationsCounter;
                    partRealisationsCounter++;
                    part.layerId=layerNo+2;
                    part.location = octetsImage[i][j]->filterPos[1][1];
                    part.locationEucl = octetsImage[i][j]->partsPosEucl[1][1];
                    //part.gaussians = Octet.o .gaussians;
                    part.partIds = octetsImage[i][j].get()->partIds;
                    part.offsets = octetsImage[i][j]->offsets;
                    part.partsPosNorm = octetsImage[i][j]->partsPosNorm;
                    part.group.clear();
                    //part.locationEucl = octetsImage[i][j]->partsPosNorm[1][1].mean.block<3,1>(0,0);
                    part.gaussians[1][1].mean=Vec3(octetsImage[i][j]->filterPos[1][1].u, octetsImage[i][j]->filterPos[1][1].v, octetsImage[i][j]->filterPos[1][1].depth);
                    if (distance<distThreshold){
                        part.id = id;
                        oldParts.push_back(part);
                    }
                    else
                        newParts.push_back(part);
                    if (octetsImage[i][j].get()->secondOctet.size()>0){
                        ViewDependentPart secondPart;
                        if (!octetsImage[i][j].get()->secondOctet[0].isBackground){
                            double distParts;
                            id = findId(hierarchy, layerNo, octetsImage[i][j].get()->secondOctet[0], distParts, offset);
                            secondPart = hierarchy.viewDependentLayers[layerNo][id];
                            secondPart.offset = offset;
                            secondPart.realisationId = partRealisationsCounter;
                            partRealisationsCounter++;
                            secondPart.layerId=layerNo+2;
                            secondPart.location = octetsImage[i][j].get()->secondOctet[0].filterPos[1][1];
                            secondPart.locationEucl = octetsImage[i][j].get()->secondOctet[0].partsPosEucl[1][1];
                            secondPart.gaussians[1][1].mean=Vec3(octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].u, octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].v, octetsImage[i][j].get()->secondOctet[0].filterPos[1][1].depth);
                            secondPart.offsets = octetsImage[i][j].get()->secondOctet[0].offsets;
                            //part.gaussians = Octet.o .gaussians;
                            secondPart.partIds = octetsImage[i][j].get()->partIds;
                            secondPart.partsPosNorm = octetsImage[i][j]->partsPosNorm;
                            secondPart.group.clear();
                            if (distParts<distThreshold){
                                secondPart.id = id;
                                oldParts.push_back(secondPart);
                            }
                            else
                                newParts.push_back(secondPart);
                        }
                    }
                }
            }
        }
    }
}

/// get last view dependent layer parts from the image
void NormalImageFilter::getLayerParts(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts, bool inference) const{
    parts.clear();
    const PartsImages* partsImgs;
    if (inference)
        partsImgs = &partsImagesInference;
    else
        partsImgs = &partsImages;
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        for (size_t i=0; i<(*partsImgs)[layerNo][overlapNo][categoryNo][objectNo][imageNo].size();i++){
            for (size_t j=0; j<(*partsImgs)[layerNo][overlapNo][categoryNo][objectNo][imageNo].back().size();j++){
                if ((*partsImgs)[layerNo][overlapNo][categoryNo][objectNo][imageNo][i][j].get()!=nullptr){
                    if (!(*partsImgs)[layerNo][overlapNo][categoryNo][objectNo][imageNo][i][j]->isBackground()){
                        parts.push_back(*(*partsImgs)[layerNo][overlapNo][categoryNo][objectNo][imageNo][i][j]);
                    }
                }
            }
        }
    }
}

/// Fill in octet
int NormalImageFilter::fillInOctet(const OctetsImage& octetsImage, const PartsImage& partsImage, int u, int v, Octet& octet) const{
    int elementsNo=0;
    for (int i=-1;i<2;i++){
        for (int j=-1;j<2;j++){
            Mat34 offset(Mat34::Identity());
            int id(-1);
            if (octetsImage[u+i][v+j].get()!=nullptr){
                if (partsImage[u+i][v+j].get()!=nullptr){
//                    if(partsImage[u+i][v+j].get()->secondVDPart.size()>0){
//                        std::cout <<"secon octet\n";
//                    }
                    id = partsImage[u+i][v+j].get()->id;
                    offset = partsImage[u+i][v+j].get()->offset;
                }
            }
            else
                id = -1;
            octet.offsets[i+1][j+1]=offset;
            octet.partIds[i+1][j+1]=id;
            if (id < 0){
                octet.filterPos[i+1][j+1].u=(v+j)*(config.filterSize*3)+((config.filterSize*3)/2);//octetsImage[u+i][v+j].filterPos[1][1];
                octet.filterPos[i+1][j+1].v=(u+i)*(config.filterSize*3)+((config.filterSize*3)/2);
                if (config.useEuclideanCoordinates)
                    sensorModel.getPoint(octet.filterPos[i+1][j+1].u, octet.filterPos[i+1][j+1].u, 1.0, octet.partsPosEucl[i+1][j+1]);
                octet.filterPos[i+1][j+1].depth = octet.partsPosEucl[i+1][j+1](2);
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=octet.partsPosEucl[i+1][j+1];
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=Vec3(0,0,1);
            }
            else {
                elementsNo++;
                octet.filterPos[i+1][j+1]=octetsImage[u+i][v+j]->filterPos[1][1];
                octet.partsPosEucl[i+1][j+1]=octetsImage[u+i][v+j]->partsPosEucl[1][1];
                //if (config.useEuclideanCoordinates)
                    //sensorModel.getPoint(octet.filterPos[i+1][j+1].u, octet.filterPos[i+1][j+1].v, octet.filterPos[i+1][j+1].depth, octet.partsPosEucl[i+1][j+1]);
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=octetsImage[u+i][v+j]->partsPosNorm[1][1].mean.block<3,1>(0,0);//octet.partsPosEucl[i+1][j+1];
                /*std::cout << i << ", " << j << " " << octet.partsPosEucl[i+1][j+1].transpose() << " pos\n";
                std::cout << i << ", " << j << " " << octet.partsPosEucl[i+1][j+1].transpose() << " pos\n";
                getchar();*/
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=octetsImage[u+i][v+j]->partsPosNorm[1][1].mean.block<3,1>(3,0);//dictionary[id].partsPosNorm[1][1].mean.block<3,1>(3,0);
                //octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=(offset*Vec4(octet.partsPosEucl[i+1][j+1](0),octet.partsPosEucl[i+1][j+1](1),octet.partsPosEucl[i+1][j+1](2),1.0)).block<3,1>(0,3);
                //octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=offset.rotation()*dictionary[id].partsPosNorm[1][1].mean.block<3,1>(3,0);
            }
        }
    }
    return elementsNo;
}

/// determine id of the part using dictionary
/*int NormalImageFilter::findId(const ViewDependentPart::Seq& dictionary, const Octet& octet) const{
    if (octet.isBackground)//background
        return -1;
    int id=0;
    for (auto & part : dictionary){
        if (part.partIds==octet.partIds){
            return id;
        }
        for (auto & word : part.group){
            if (word.partIds==octet.partIds){
                return id;
            }
        }
        id++;
    }
    std::cout << "Part not found in the dictionary!\n";
    octet.print();
    getchar();
    return -1;
}*/

/// determine id of the part using dictionary
int NormalImageFilter::findId(const hop3d::Hierarchy& hierarchy, int layerNo, const Octet& octet, double& distance, Mat34& offset) const{
    if (octet.isBackground)//background
        return -1;
    int id=0; int foundId(0);
    double minDist=std::numeric_limits<double>::max();
    ViewDependentPart partVD;
    partVD.partsPosNorm = octet.partsPosNorm;
    partVD.partIds = octet.partIds;
    partVD.offsets = octet.offsets;
    Mat34 offsetTmp;
    int partID=0;
    for (auto & part : hierarchy.viewDependentLayers[layerNo]){
        double dist(0);
        if (layerNo==0)
            dist = ViewDependentPart::distanceInvariant(part, partVD, 3, offsetTmp);
        else if (layerNo==1) {
            dist = ViewDependentPart::distanceInvariant(part, partVD, 3, hierarchy.viewDependentLayers[0], offsetTmp);
        }
        else if (layerNo==2) {
            dist = ViewDependentPart::distanceInvariant(part, partVD, 3, hierarchy.viewDependentLayers[0],hierarchy.viewDependentLayers[1], offsetTmp);
        }
        partID++;
        if (dist==0){
            offset = offsetTmp;
            return id;
        }
        if (dist<minDist){
            offset=offsetTmp;
            minDist = dist;
            foundId = id;
        }
        id++;
    }
    distance = minDist;
    return foundId;
}

/// get filters
void NormalImageFilter::getFilters(Filter::Seq& _filters) const{
    _filters = this->filters;
}

void NormalImageFilter::setFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName){
    std::cout << "This method is not used for normal-based filtering " << patchesFileName << "," << normalsFileName << ", " << masksFileName << "\n";
}

/// discretization: convert normal vector to id
int NormalImageFilter::toId(const Vec3& normal) const{
    //double latitude = acos(normal(3));
    double longitude = atan2(normal(1),normal(0));
    if (longitude<0)
        longitude= 2*M_PI+longitude;
    if (longitude>2*M_PI)
        longitude = longitude-2*M_PI;
    double radius = sqrt(pow(normal(0),2.0)+pow(normal(1),2.0)); // xy radius
    //int ringId = (int)floor(radius*config.ringsNo);
    //int ringId = (int)floor(sin(radius*(M_PI/2))*config.ringsNo);
    int ringId = (int)floor(sqrt(radius)*config.ringsNo);
    double idDir = (ringId==0) ? 0 : (longitude/(2*M_PI))*(pow(2.0,ringId+1)); ///direction id
    double startId = 2*(pow(2.0,ringId)-1);
    int id = (ringId==0) ? 0 : (int)floor(startId+idDir-1);
    return id;
}

/// convert id of the filter to normal vector
void NormalImageFilter::toNormal(int id, Vec3& normal) const{
    id+=1;
    int ringId = 0;
    for (int ring=1;ring<config.ringsNo+1;ring++){
        if (id<2*(pow(2.0,ring)-1)){
            ringId=ring;
            break;
        }
    }
    double dirId = (ringId==1) ? 1 : id-2*(pow(2.0,(ringId-1))-1)+1;
    //compute normal vector
    if (ringId==1){
          normal = Vec3(0,0,1);
    }
    else {
        //double radius=(ringId-1)*(1.0/double(config.ringsNo))+(0.5/double(config.ringsNo));
        //double radius = (asin((ringId-1)*(1.0/double(config.ringsNo)))+asin((ringId)*(1.0/double(config.ringsNo))))/M_PI;
        double radius = (pow((ringId-1)*(1.0/double(config.ringsNo)),2.0)+pow((ringId)*(1.0/double(config.ringsNo)),2.0))/2.0;
        double angle = (dirId-1)*(2*M_PI/pow(2.0,(ringId)))+(M_PI/pow(2.0,(ringId)));
        normal = Vec3(radius*cos(angle), radius*sin(angle), sqrt(1-pow(radius*cos(angle),2.0)-pow(radius*sin(angle),2.0)));
    }
}

/// compute normal on the image (filter size)
void NormalImageFilter::computeNormal(int u, int v, std::vector< std::vector<hop3d::PointNormal> >& cloudOrd) const{
    std::vector<hop3d::PointNormal> points;
    double min=std::numeric_limits<double>::max();
    double max=std::numeric_limits<double>::min();
    for (int i=-config.PCAWindowSize/2;i<1+config.PCAWindowSize/2;i++){
        for (int j=-config.PCAWindowSize/2;j<1+config.PCAWindowSize/2;j++){
            if (((u+i)>=0)&&((u+i)<(int)cloudOrd.size())&&((v+j)>=0)&&(int(v+j)<(int)cloudOrd[u+i].size()))
				if (!std::isnan(double(cloudOrd[u + i][v + j].position(2)))){
                    if (cloudOrd[u+i][v+j].position(2)<min)
                        min=cloudOrd[u+i][v+j].position(2);
                    if (cloudOrd[u+i][v+j].position(2)>max)
                        max=cloudOrd[u+i][v+j].position(2);
                    points.push_back(cloudOrd[u+i][v+j]);
                   // std::cout << "point " << cloudOrd[u+i][v+j].position << "\n";
                }
        }
    }
    if (points.size()>size_t(config.PCAbackgroundThreshold)){
        if (config.PCAuseClustering){
            if ((max-min)>config.PCADistThreshold) {// try to create two clasters
                std::vector<hop3d::PointNormal> pointGroup;
                if (extractGroup(points, pointGroup)){
                    normalPCA(pointGroup, cloudOrd[u][v]);
                }
                else {
                    cloudOrd[u][v].normal = Vec3(NAN,NAN,NAN);
                }
            }
            else{
                normalPCA(points, cloudOrd[u][v]);
            }
        }
        else {
            if ((max-min)>config.PCADistThreshold)
                cloudOrd[u][v].normal = Vec3(NAN,NAN,NAN);
            else
                normalPCA(points, cloudOrd[u][v]);
        }
    }
    else {
        cloudOrd[u][v].normal = Vec3(NAN,NAN,NAN);
    }
}

/// try to extract two clusters. If clusters are well separated (PCARelDistClusters parameter) return true
bool NormalImageFilter::extractGroup(const std::vector<hop3d::PointNormal>& points, std::vector<hop3d::PointNormal>& pointGroup) const{
    int clustersNo=2;//its written for two clusters only
    std::vector<int> centroids={0,1};
    std::vector<std::vector<int>> clusters; clusters.resize(clustersNo);
    int iterMax=20;
    for (int i=0;i<iterMax;i++){
        //assign to clusters
        int pointNo=0;
        clusters[0].clear(); clusters[1].clear();//clear both clusters
        for (auto& point : points){
            if (pointNo==centroids[0])
                clusters[0].push_back(pointNo);
            else if (pointNo==centroids[1])
                clusters[1].push_back(pointNo);
            else{
                if (fabs(points[centroids[0]].position(2)-point.position(2))<fabs(points[centroids[1]].position(2)-point.position(2)))
                    clusters[0].push_back(pointNo);
                else
                    clusters[1].push_back(pointNo);
            }
            pointNo++;
        }
        // find new centroids
        for (int j=0;j<clustersNo;j++){
            double minDist[2]={std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
            for (auto& point : clusters[j]){
                double dist = 0;
                for (auto& point2 : clusters[j]){
                    dist+=fabs(points[point].position(2)-points[point2].position(2));
                }
                dist = dist/double(clusters[j].size());
                if (dist<minDist[j]){
                    minDist[j]=dist;
                    centroids[j]=point;
                }
            }
        }
    }
    //computer parameters of two clusters
    double minDist[2]={std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    double maxDist[2]={std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    for (int j=0;j<clustersNo;j++){
        for (auto& partId : clusters[j]){
            if (points[partId].position(2)>maxDist[j])
                maxDist[j]=points[partId].position(2);
            if (points[partId].position(2)<minDist[j])
                minDist[j]=points[partId].position(2);
        }
    }
    double dist = (minDist[1]>minDist[0]) ? (minDist[1]-maxDist[0]) : (minDist[0]-maxDist[1]);
    double width = (minDist[1]>minDist[0]) ? (maxDist[0]-minDist[0]) : (maxDist[1]-minDist[1]);
    int bigger, smaller;
    if (clusters[0].size()>clusters[1].size()){
        bigger=0; smaller=1;
    }
    else{
        bigger=1; smaller=0;
    }
    if (clusters[bigger].size()<=(size_t)config.minOctetSize)
        return false;
    if ((double(clusters[smaller].size())/double(clusters[bigger].size()))<0.75)
        return false;

    if (fabs(dist/width)>config.PCARelDistClusters){
        int clusterNo = bigger;//(minDist[1]>minDist[0]) ? 0 : 1;
        pointGroup.clear();
        for (auto& id : clusters[clusterNo])
            pointGroup.push_back(points[id]);
        return true;
        //return false;// we don't want groups
    }
    return false;
}

/// Compute normal vector using PCA
void NormalImageFilter::normalPCA(std::vector<hop3d::PointNormal>& points, hop3d::PointNormal& pointNormal) const{
    //compute mean
    Vec3 mean(0,0,0);
    for (auto & point : points){
        mean+=point.position;
    }
    mean/=(double)points.size();
    // compute covariance
    Mat33 cov(Mat33::Zero());
    for (auto & point : points){
        cov+=(point.position-mean)*(point.position-mean).transpose();
    }
    Eigen::EigenSolver<Mat33> es(cov);
/*    std::cout << "cov: \n" << cov << "\n";
    std::cout << "eigenvalues \n" << es.eigenvalues() << "\n";
    std::cout << "eigenvector0 " << es.eigenvectors() << "\n";
    std::cout << std::real(es.eigenvectors()(0,0)) << ", " << std::real(es.eigenvectors()(1,0)) << ", " << std::real(es.eigenvectors()(2,0)) << "\n";
*/
    int min=0;
    if (std::real(es.eigenvalues()(1))<std::real(es.eigenvalues()(0))){
        min=1;
        if (std::real(es.eigenvalues()(2))<std::real(es.eigenvalues()(1)))
            min=2;
    }
    else if (std::real(es.eigenvalues()(2))<std::real(es.eigenvalues()(1))){
        min=2;
        if (std::real(es.eigenvalues()(0))<std::real(es.eigenvalues()(2)))
            min=0;
    }
    pointNormal.normal = Vec3(std::real(es.eigenvectors()(1,min)), std::real(es.eigenvectors()(0,min)), std::real(es.eigenvectors()(2,min)));
    if (pointNormal.normal(2)<0)
        pointNormal.normal=-pointNormal.normal;
    //std::cout << "normal " << pointNormal.normal.transpose() << "\n";
    //getchar();
}

/// Generate filters
void NormalImageFilter::generateFilters(void){
    int filtersNo=2*int(pow(2.0,config.ringsNo)-1)-1;
    filters.resize(filtersNo);
    for (size_t i=0;i<(size_t)filtersNo;i++){
        cv::Mat patchTmp(config.filterSize,config.filterSize, cv::DataType<double>::type);
        toNormal(int(i),filters[i].normal);
        Mat33 coord(coordinateFromNormal(filters[i].normal));
        Mat34 rot(Mat34::Identity());
        rot.matrix().block<3,3>(0,0)=coord;
        int u=0;
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            int v=0;
            for (int k=-config.filterSize/2;k<1+config.filterSize/2;k++){
                Mat34 point(Mat34::Identity());
                point(0,3) = double(j); point(1,3) = double(k);
                point=rot*point;
                patchTmp.at<double>(u,v) = point(2,3);
                v++;
            }
            u++;
        }
        cv::Mat maskTmp(config.filterSize,config.filterSize, cv::DataType<double>::type,1);
        filters[i].id=(int)i;
        filters[i].patch = patchTmp.clone();
        filters[i].mask = maskTmp.clone();
    }
}

/// compute coordinate system from normal vector
Mat33 NormalImageFilter::coordinateFromNormal(const Vec3& _normal){
    Vec3 x(1,0,0); Vec3 y;
    Vec3 normal(_normal);
    y = normal.cross(x);
    y.normalize();
    x = y.cross(normal);
    x.normalize();
    Mat33 R;
    R.block(0,0,3,1) = x;
    R.block(0,1,3,1) = y;
    R.block(0,2,3,1) = normal;
    return R;
}

/// normalize vector
/*void NormalImageFilter::normalizeVector(Vec3& normal){
    double norm = normal.norm();
    normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
}*/

/// get set of ids for the given input point
void NormalImageFilter::getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference){
    OctetsImages* oImages;
    PartsImages* pImages;
    if (inference){
        oImages = &octetsImagesInference;
        pImages = &partsImagesInference;
    }
    else {
        oImages = &octetsImages;
        pImages = &partsImages;
    }
    unsigned int octetCoords[2]={(u-overlapNo*config.filterSize)/(config.filterSize*3),(v-overlapNo*config.filterSize)/(config.filterSize*3)};
    if ((octetCoords[0]>0)&&(octetCoords[1]>0)&&(octetCoords[0]<(*oImages)[0][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords[1]<(*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            Octet octet = *((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]);
            if (octet.partIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]==-2){
                if (octet.secondOctet.size()>0)
                    ids.push_back(octet.secondOctet[0].partIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]);
                else
                    ids.push_back(-2);
            }
            else {
                ids.push_back(octet.partIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]);
            }
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part
    }
    if (octetCoords[0]<(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo].size()&&octetCoords[1]<(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][0].size()){
        if ((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            if (!(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->isBackground()){
                if ((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
                    //ViewDependentPart.partsPosNorm[1][1]
                    Octet * octet = (*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get();
                    if (octet->partIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]==-2){
                        if (octet->secondOctet.size()>0){
                            if (fabs(octet->secondOctet[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->secondVDPart[0].id);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->id);
                    }
                }
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2);
    }
    unsigned int octetCoords2nd[2]={(u-overlapNo*(3*config.filterSize))/(config.filterSize*3*3),(v-overlapNo*(3*config.filterSize))/(config.filterSize*3*3)};
    if ((octetCoords2nd[0]>0)&&(octetCoords2nd[1]>0)&&(octetCoords2nd[0]<(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords2nd[1]<(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
            if (!(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->isBackground()){
                if ((*oImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
                    //ViewDependentPart.partsPosNorm[1][1]
                    Octet * octet = (*oImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get();
                    if (octet->partIds[((u-overlapNo*(3*config.filterSize))/(config.filterSize*3))%3][((v-overlapNo*(3*config.filterSize))/(config.filterSize*3))%3]<-1){
                        if (octet->secondOctet.size()>0){
                            if (fabs(octet->secondOctet[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->secondVDPart[0].id);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->id);
                    }
                }
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
    }
    unsigned int octetCoords3rd[2]={(u-(overlapNo*(3*3*config.filterSize)))/(config.filterSize*3*3*3),(v-(overlapNo*(3*3*config.filterSize)))/(config.filterSize*3*3*3)};
    /*std::cout << "octetCoords2nd[2] " << octetCoords2nd[0] << " - " << octetCoords2nd[1] << "\n";
    std::cout << "octetCoords3rd[2] " << octetCoords3rd[0] << " - " << octetCoords3rd[1] << "\n";
    for (int over=0;over<3;over++){
        std::cout << "overlap lay prev="  << over << "\n";
        for (auto & row : (*pImages)[1][over][categoryNo][objectNo][imageNo]){
            for (auto & part : row){
                if (part.get()!=nullptr){
                    if(!part->isBackground()){
                        std::cout << part->id << " ";
                    }
                    else
                        std::cout << "-2 ";
                }
                else
                    std::cout << "-2 ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }
    for (int over=0;over<3;over++){
        std::cout << "overlap="  << over << "\n";
        for (auto & row : (*pImages)[2][over][categoryNo][objectNo][imageNo]){
            for (auto & part : row){
                if (part.get()!=nullptr){
                    if(!part->isBackground()){
                        std::cout << part->id << " ";
                    }
                    else
                        std::cout << "-2 ";
                }
                else
                    std::cout << "-2 ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }*/
    if ((octetCoords3rd[0]>0)&&(octetCoords3rd[1]>0)&&(octetCoords3rd[0]<(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords3rd[1]<(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get()!=nullptr){
            if (!(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->isBackground()){
               // if ((*oImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get()!=nullptr){
                    //Octet * octet = (*oImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get();
                    ViewDependentPart* octet = (*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get();
                    if (octet->partIds[((u-overlapNo*(3*3*config.filterSize))/(config.filterSize*3*3))%3][((v-overlapNo*(3*3*config.filterSize))/(config.filterSize*3*3))%3]<-1){
                        if (octet->secondVDPart.size()>0){
                            if (fabs(octet->secondVDPart[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer+config.distThresholdThirdLayer))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->secondVDPart[0].id);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer+config.distThresholdThirdLayer))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->id);
                    }
                /*}
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }*/
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
    }
}

/// get set of ids for the given input point
void NormalImageFilter::getRealisationsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, double depth, std::vector<int>& ids, bool inference){
    OctetsImages* oImages;
    PartsImages* pImages;
    if (inference){
        oImages = &octetsImagesInference;
        pImages = &partsImagesInference;
    }
    else {
        oImages = &octetsImages;
        pImages = &partsImages;
    }
    unsigned int octetCoords[2]={(u-overlapNo*config.filterSize)/(config.filterSize*3),(v-overlapNo*config.filterSize)/(config.filterSize*3)};
    if ((octetCoords[0]>0)&&(octetCoords[1]>0)&&(octetCoords[0]<(*oImages)[0][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords[1]<(*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            Octet octet = *((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]);
            if (octet.partIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]==-2){
                if (octet.secondOctet.size()>0)
                    ids.push_back(octet.secondOctet[0].realisationsIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]);
                else
                    ids.push_back(-2);
            }
            else {
                ids.push_back(octet.realisationsIds[((u-overlapNo*config.filterSize)/(config.filterSize))%3][((v-overlapNo*config.filterSize)/(config.filterSize))%3]);
            }
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part
    }
    if (octetCoords[0]<(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo].size()&&octetCoords[1]<(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][0].size()){
        if ((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            if (!(*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->isBackground()){
                if ((*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
                    //ViewDependentPart.partsPosNorm[1][1]
                    Octet * octet = (*oImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get();
                    if (octet->partIds[(u/(config.filterSize))%3][(v/(config.filterSize))%3]==-2){
                        if (octet->secondOctet.size()>0){
                            if (fabs(octet->secondOctet[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->secondVDPart[0].realisationId);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[0][overlapNo][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->realisationId);
                    }
                }
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2);
    }
    unsigned int octetCoords2nd[2]={u/(config.filterSize*3*3),v/(config.filterSize*3*3)};
    if ((octetCoords2nd[0]<(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords2nd[1]<(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
            if (!(*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->isBackground()){
                if ((*oImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
                    //ViewDependentPart.partsPosNorm[1][1]
                    Octet * octet = (*oImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get();
                    if (octet->partIds[(u/(config.filterSize*3))%3][(v/(config.filterSize*3))%3]==-2){
                        if (octet->secondOctet.size()>0){
                            if (fabs(octet->secondOctet[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->secondVDPart[0].realisationId);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[1][overlapNo][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->realisationId);
                    }
                }
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
    }
    unsigned int octetCoords3rd[2]={(u-(overlapNo*(3*3*config.filterSize)))/(config.filterSize*3*3*3),(v-(overlapNo*(3*3*config.filterSize)))/(config.filterSize*3*3*3)};
    if ((octetCoords3rd[0]>0)&&(octetCoords3rd[1]>0)&&(octetCoords3rd[0]<(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo].size())&&(octetCoords3rd[1]<(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][0].size())){
        if ((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get()!=nullptr){
            if (!(*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->isBackground()){
               // if ((*oImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get()!=nullptr){
                    //Octet * octet = (*oImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get();
                    ViewDependentPart* octet = (*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]].get();
                    if (octet->partIds[((u-overlapNo*(3*3*config.filterSize))/(config.filterSize*3*3))%3][((v-overlapNo*(3*3*config.filterSize))/(config.filterSize*3*3))%3]<-1){
                        if (octet->secondVDPart.size()>0){
                            if (fabs(octet->secondVDPart[0].partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer+config.distThresholdThirdLayer))
                                ids.push_back(-2);
                            else
                                ids.push_back((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->secondVDPart[0].realisationId);
                        }
                        else
                            ids.push_back(-2);
                    }
                    else {
                        if (fabs(octet->partsPosNorm[1][1].mean(2)-depth)>(config.PCADistThreshold+config.distThresholdSecondLayer+config.distThresholdThirdLayer))
                            ids.push_back(-2);
                        else
                            ids.push_back((*pImages)[2][overlapNo][categoryNo][objectNo][imageNo][octetCoords3rd[0]][octetCoords3rd[1]]->realisationId);
                    }
                /*}
                else {
                    std::cout <<"something is wrong with octets image\n";
                    getchar();
                }*/
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
    }
}

/// Insertion operator
/*std::ostream& operator<<(std::ostream& os, const NormalImageFilter& filter){
    filter.save2file(os, inference);
    return os;
}*/

/// merge octets and parts images
void NormalImageFilter::mergeTrainAndInfResults(void){
    size_t layerNo=0;
    for (auto &layer : octetsImagesInference){
        size_t overlapNo=0;
        for (auto &overlap : layer){
            auto& overlapTrain = octetsImages[layerNo];
            size_t categoryNo=0;
            for (auto &category : overlap){
                auto& categoryTrain = overlapTrain[overlapNo];
                categoryTrain.resize(categoryTrain.size()+1);
                size_t objectNo=0;
                for (auto &object : category){
                    auto& objectTrain = categoryTrain[categoryTrain.size()-1];
                    objectTrain.resize(objectTrain.size()+1);
                    size_t imageNo=0;
                    for (auto &image : object){
                        auto& imageTrain = objectTrain[objectTrain.size()-1];
                        imageTrain.push_back(image);
                        imageNo++;
                    }
                    objectNo++;
                }
                categoryNo++;
            }
            overlapNo++;
        }
        layerNo++;
    }
    layerNo=0;
    for (auto &layer : partsImagesInference){
        size_t overlapNo=0;
        for (auto &overlap : layer){
            auto& overlapTrain = partsImages[layerNo];
            size_t categoryNo=0;
            for (auto &category : overlap){
                auto& categoryTrain = overlapTrain[overlapNo];
                categoryTrain.resize(categoryTrain.size()+1);
                size_t objectNo=0;
                for (auto &object : category){
                    auto& objectTrain = categoryTrain[categoryTrain.size()-1];
                    objectTrain.resize(objectTrain.size()+1);
                    size_t imageNo=0;
                    for (auto &image : object){
                        auto& imageTrain = objectTrain[objectTrain.size()-1];
                        imageTrain.push_back(image);
                        imageNo++;
                    }
                    objectNo++;
                }
                categoryNo++;
            }
            overlapNo++;
        }
        layerNo++;
    }
}

/// save to file
void NormalImageFilter::save2file(std::ostream& os, bool inference) const{
    OctetsImages oImages;
    if (inference)
        oImages = octetsImagesInference;
    else
        oImages = octetsImages;
    os << oImages.size() << " ";
    for (auto &layer : oImages){
        os << layer.size() << " ";
        for (auto &overlap : layer){
            os << overlap.size() << " ";
            for (auto &category : overlap){
                os << category.size() << " ";
                for (auto &object : category){
                    os << object.size() << " ";
                    for (auto &image : object){
                        os << image.size() << " ";
                        for (auto &row : image){
                            os << row.size() << " ";
                            for (auto &octet : row){
                                if (octet.get()!=nullptr){
                                    os << 1 << " ";
                                    os << *octet;
                                }
                                else
                                    os << 0 << " ";
                            }
                        }
                    }
                }
            }
        }
    }
    PartsImages pImages;
    if (inference)
        pImages = partsImagesInference;
    else
        pImages = partsImages;
    os << pImages.size() << " ";
    for (auto &layer : pImages){
        os << layer.size() << " ";
        for (auto &overlap : layer){
            os << overlap.size() << " ";
            for (auto &category : overlap){
                os << category.size() << " ";
                for (auto &object : category){
                    os << object.size() << " ";
                    for (auto &image : object){
                        os << image.size() << " ";
                        for (auto &row : image){
                            os << row.size() << " ";
                            for (auto &part : row){
                                if (part.get()!=nullptr){
                                    os << 1 << " ";
                                    os << *part << " ";
                                }
                                else{
                                    os << 0 << " ";
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    os << "\n";
}

// Extraction operator
/*std::istream& operator>>(std::istream& is, NormalImageFilter& filter){
    filter.loadFromfile(is);
    return is;
}*/

/// load from file
void NormalImageFilter::loadFromfile(std::istream& is, bool inference){
    // read categories no
    int layersNo;
    is >> layersNo;
    OctetsImages* oImages;
    if (inference)
        oImages = &octetsImagesInference;
    else
        oImages = &octetsImages;
    oImages->clear();
    oImages->resize(layersNo);
    for (int layNo = 0; layNo<layersNo; layNo++){
        int overlapsNo;
        is >> overlapsNo;
        (*oImages)[layNo].resize(overlapsNo);
        for (int overlapNo = 0; overlapNo<overlapsNo; overlapNo++){
            int categoriesNo;
            is >> categoriesNo;
            (*oImages)[layNo][overlapNo].resize(categoriesNo);
            for (int catNo = 0; catNo<categoriesNo; catNo++){
                int objectsNo;
                is >> objectsNo;
                (*oImages)[layNo][overlapNo][catNo].resize(objectsNo);
                for (int objNo=0;objNo<objectsNo; objNo++){
                    int imgsNo;
                    is >> imgsNo;
                    (*oImages)[layNo][overlapNo][catNo][objNo].resize(imgsNo);
                    for (int imgNo=0;imgNo<imgsNo;imgNo++){
                        int rowsNo;
                        is >> rowsNo;
                        (*oImages)[layNo][overlapNo][catNo][objNo][imgNo].resize(rowsNo);
                        for (int rowNo=0;rowNo<rowsNo;rowNo++){
                            int octetsNo;
                            is >> octetsNo;
                            (*oImages)[layNo][overlapNo][catNo][objNo][imgNo][rowNo].resize(octetsNo);
                            for (int octetNo=0;octetNo<octetsNo;octetNo++){
                                int isOctet;
                                is >> isOctet;
                                if (isOctet==1){
                                    Octet octet;
                                    is >> octet;
                                    (*oImages)[layNo][overlapNo][catNo][objNo][imgNo][rowNo][octetNo].reset(new Octet(octet));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    is >> layersNo;
    PartsImages* pImages;
    if (inference)
        pImages = &partsImagesInference;
    else
        pImages = &partsImages;
    pImages->clear();
    pImages->resize(layersNo);
    for (int layNo = 0; layNo<layersNo; layNo++){
        int overlapsNo;
        is >> overlapsNo;
        (*pImages)[layNo].resize(overlapsNo);
        for (int overlapNo = 0; overlapNo<overlapsNo; overlapNo++){
            int catsNo;
            is >> catsNo;
            (*pImages)[layNo][overlapNo].resize(catsNo);
            for (int catNo = 0; catNo<catsNo; catNo++){
                int objectsNo;
                is >> objectsNo;
                (*pImages)[layNo][overlapNo][catNo].resize(objectsNo);
                for (int objNo=0;objNo<objectsNo; objNo++){
                    int imgsNo;
                    is >> imgsNo;
                    (*pImages)[layNo][overlapNo][catNo][objNo].resize(imgsNo);
                    for (int imgNo=0;imgNo<imgsNo;imgNo++){
                        int rowsNo;
                        is >> rowsNo;
                        (*pImages)[layNo][overlapNo][catNo][objNo][imgNo].resize(rowsNo);
                        for (int rowNo=0;rowNo<rowsNo;rowNo++){
                            int partsNo;
                            is >> partsNo;
                            (*pImages)[layNo][overlapNo][catNo][objNo][imgNo][rowNo].resize(partsNo);
                            for (int partNo=0;partNo<partsNo;partNo++){
                                int isPart;
                                is >> isPart;
                                if (isPart==1){
                                    ViewDependentPart part;
                                    is >> part;
                                    (*pImages)[layNo][overlapNo][catNo][objNo][imgNo][rowNo][partNo].reset(new ViewDependentPart(part));

                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/// get numbers of realisations
int NormalImageFilter::getRealisationsNo(void) const{
    return partRealisationsCounter;
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(void) {
    filterNorm.reset(new NormalImageFilter());
    return filterNorm.get();
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(std::string config, std::string sensorConfig) {
    filterNorm.reset(new NormalImageFilter(config, sensorConfig));
    return filterNorm.get();
}
