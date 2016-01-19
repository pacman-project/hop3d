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
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("useEuclideanCoordinates", &useEuclideanCoordinates);

    model->FirstChildElement( "PCA" )->QueryIntAttribute("PCAWindowSize", &PCAWindowSize);
    model->FirstChildElement( "PCA" )->QueryDoubleAttribute("PCADistThreshold", &PCADistThreshold);
    model->FirstChildElement( "PCA" )->QueryDoubleAttribute("PCARelDistClusters", &PCARelDistClusters);
    model->FirstChildElement( "PCA" )->QueryBoolAttribute("PCAuseClustering", &PCAuseClustering);
    model->FirstChildElement( "PCA" )->QueryIntAttribute("PCAbackgroundThreshold", &PCAbackgroundThreshold);

    model->FirstChildElement( "imageFiltering" )->QueryBoolAttribute("useMedianFilter", &useMedianFilter);
    model->FirstChildElement( "imageFiltering" )->QueryIntAttribute("kernelSize", &kernelSize);

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
    for (int i=0;i<filteredImg.rows;i++){
        for (int j=0;j<filteredImg.cols;j++){
            PointNormalUV puv;
            sensorModel.getPoint(j, i, filteredImg.at<uint16_t>(i,j)*scale, cloudOrd[i][j].position);
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
                computeNormal(i, j, cloudOrd);
                cloud[pointIdx].normal = cloudOrd[i][j].normal;
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
            if ((i>=0)&&(j>=0)&&(i<inputImg.rows)&&(j<inputImg.cols)){
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
}

///compute set of octets from set of the depth images
void NormalImageFilter::computeOctets(const cv::Mat& depthImage, int categoryNo, int objectNo, int imageNo, hop3d::Octet::Seq& octets){
    octets.clear();
    /*int filtersNo=2*int(pow(2.0,config.ringsNo)-1)-1;
    for (int i=0;i<filtersNo;i++){
        Vec3 normal;
        this->toNormal(i,normal);
        std::cout << "set id " << i << "\n";
        std::cout << "computed normal " << normal.transpose() << "\n";
        std::cout << "computed id " << this->toId(normal) << "\n";
        getchar();
    }
    std::cout << "number of filters: " << filtersNo << "\n";
    getchar();*/
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
    for (int i=0;i<filteredImg.rows;i++){
        for (int j=0;j<filteredImg.cols;j++){
            sensorModel.getPoint(i, j, filteredImg.at<uint16_t>(i,j)*scale, cloudOrd[i][j].position);
            /*if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                PointNormalUV puv;
                Vec3 p3d;
                sensorModel.getPoint(j, i, filteredImg.at<uint16_t>(i,j)*scale, p3d);
                puv.position = p3d;
                puv.u=i; puv.v=j;
                cloudGrasp.push_back(puv);
            }*/
        }
    }
    int pointIdx = 0;
    cv::Mat idsImage(filteredImg.rows,filteredImg.cols, cv::DataType<int>::type,cv::Scalar(0));
    for (int i=config.filterSize/2;i<filteredImg.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<filteredImg.cols-(config.filterSize/2);j++){
            if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                computeNormal(i, j, cloudOrd);
                //cloudGrasp[pointIdx].normal = cloudOrd[i][j].normal;
                //compute id 
				if (!std::isnan(double(cloudOrd[i][j].normal(2)))){
                    responseImage[i][j].first = toId(cloudOrd[i][j].normal);
                    //compute response (dot product)
                    responseImage[i][j].second = cloudOrd[i][j].normal.adjoint()*filters[responseImage[i][j].first].normal;
                    /*std::cout << "normal " << cloudOrd[i][j].normal.transpose() << "\n";
                    std::cout << "filter normal " << filters[responseImage[i][j].first].normal.transpose() << "\n";
                    std::cout << "[" << i << ", " << j << "] " << responseImage[i][j].first << " -> " << responseImage[i][j].second << "\n";
                    getchar();*/
                    if (config.verbose==2)
                        idsImage.at<uchar>(i,j)=(uchar)(toId(cloudOrd[i][j].normal)*2);
                }
                pointIdx++;
            }
        }
    }
    /*for (size_t i=0;i<responseImage.size();i++){
        for (size_t j=0;j<responseImage[i].size();j++){
            if (responseImage[i][j].first!=-1){
                std::cout << "[" << i << ", " << j << "] " << responseImage[i][j].first << " -> " << responseImage[i][j].second << "\n";
                std::cout << cloudOrd[i][j].position(2) << "\n";
                getchar();
            }
        }
    }*/
    OctetsImage octetsImage = extractOctets(responseImage, cloudOrd, octets);
    updateOctetsImage(0,categoryNo, objectNo, imageNo, octetsImage);
    //saveCloud(categoryNo, objectNo, imageNo, cloudGrasp);

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
void NormalImageFilter::updateOctetsImage(int layerNo, int categoryNo, int objectNo, int imageNo, const OctetsImage& octetsImage){
    if((int)octetsImages.size()<=layerNo){
        octetsImages.resize(layerNo+1);
    }
    if((int)octetsImages[layerNo].size()<=categoryNo){
        octetsImages[layerNo].resize(categoryNo+1);
    }
    if ((int)octetsImages[layerNo][categoryNo].size()<=objectNo){
        octetsImages[layerNo][categoryNo].resize(objectNo+1);
    }
    if ((int)octetsImages[layerNo][categoryNo][objectNo].size()<=imageNo){
        octetsImages[layerNo][categoryNo][objectNo].resize(imageNo+1);
    }
    octetsImages[layerNo][categoryNo][objectNo][imageNo] = octetsImage;
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
void NormalImageFilter::getResponseFilters(int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords) const {
    partCoords.clear();
    for (auto& row : octetsImages[0][categoryNo][objectNo][imageNo]){
        for (auto& octet : row){
            if (octet.get()!=nullptr){
                if (!octet->isBackground){
                    for (int i=0;i<3;i++){
                        for (int j=0;j<3;j++){
                            if (octet->partIds[i][j]!=-1){
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

/// returs parts ids and their position on the image
void NormalImageFilter::getPartsRealisation(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts) const{
    parts.clear();
    if (layerNo==0){
        for (size_t row=0; row<octetsImages[layerNo][categoryNo][objectNo][imageNo].size();row++){
            for (size_t col=0; col<octetsImages[layerNo][categoryNo][objectNo][imageNo][row].size();col++){
                if (octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col].get()!=nullptr){
                    if (!octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col]->isBackground){
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                if (octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col]->partIds[i][j]>=0){
                                    ViewDependentPart vdp;
                                    vdp.locationEucl = octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col]->partsPosEucl[1][1]+octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col]->partsPosEucl[i][j];
                                    vdp.offset=Mat34::Identity();
                                    vdp.id = 0;
                                    vdp.realisationId = octetsImages[layerNo][categoryNo][objectNo][imageNo][row][col]->realisationsIds[i][j];
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
        for (auto& row : partsImages[layerNo-1][categoryNo][objectNo][imageNo]){
            for (auto& part : row){
                if (part.get()!=nullptr){
                    if (!part->isBackground()){
                        parts.push_back(*part);
                    }
                }
            }
        }
    }
}

/// returs parts ids and their position on the image
void NormalImageFilter::getParts3D(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoords>& partCoords) const{
    partCoords.clear();
    if (layerNo==1){
        for (auto& row : octetsImages[1][categoryNo][objectNo][imageNo]){
            for (auto& octet : row){
                if (octet.get()!=nullptr){
                    if (!octet->isBackground){
                        for (int i=0;i<3;i++){
                            for (int j=0;j<3;j++){
                                if (octet->partIds[i][j]!=-1){
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
        for (auto& row : partsImages[layerNo-1][categoryNo][objectNo][imageNo]){
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
void NormalImageFilter::getParts3D(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<PartCoordsEucl>& partCoords) const{
    partCoords.clear();
    if (layerNo>0){
        for (auto& row : partsImages[layerNo-1][categoryNo][objectNo][imageNo]){
            for (auto& part : row){
                if (part.get()!=nullptr){
                    if (!part->isBackground()){
                        PartCoordsEucl fcoords(part->id, part->locationEucl, part->offset);
                        partCoords.push_back(fcoords);
                    }
                }
            }
        }
    }
}

/// update structure which holds parts images
void NormalImageFilter::updatePartsImages(int categoryNo, int objectNo, int imageNo, int layerNo, const PartsImage& partsImage){
    if((int)partsImages.size()<=layerNo){
        partsImages.resize(layerNo+1);
    }
    if((int)partsImages[layerNo].size()<=categoryNo){
        partsImages[layerNo].resize(categoryNo+1);
    }
    if ((int)partsImages[layerNo][categoryNo].size()<=objectNo){
        partsImages[layerNo][categoryNo].resize(objectNo+1);
    }
    if ((int)partsImages[layerNo][categoryNo][objectNo].size()<=imageNo){
        partsImages[layerNo][categoryNo][objectNo].resize(imageNo+1);
    }
    partsImages[layerNo][categoryNo][objectNo][imageNo] = partsImage;
}

///extract octets from response image
NormalImageFilter::OctetsImage NormalImageFilter::extractOctets(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, hop3d::Octet::Seq& octets){
    int u=0;
    OctetsImage octetsImage(responseImg.size()/(config.filterSize*3), std::vector<std::shared_ptr<Octet>> (responseImg[0].size()/(config.filterSize*3)));
    for (size_t i=config.filterSize+config.filterSize/2;i<responseImg.size()-config.filterSize-(config.filterSize/2);i=i+3*config.filterSize){
        int v=0;
        for (size_t j=config.filterSize+config.filterSize/2;j<responseImg[0].size()-config.filterSize-(config.filterSize/2);j=j+3*config.filterSize){
            Octet octet;
            if (computeOctet(responseImg, cloudOrd, int(i), int(j), octet)){
                for (int k=0;k<(int)octet.realisationsIds.size();k++){//update realisations ids
                    for (int l=0;l<(int)octet.realisationsIds.size();l++){
                        if (octet.partIds[k][l]>=0){
                            octet.realisationsIds[k][l]=partRealisationsCounter;
                            partRealisationsCounter++;
                        }
                    }
                }
                octetsImage[u][v].reset(new Octet(octet));
                octets.push_back(octet);
                /*octet.print();
                getchar();*/
            }
            v++;
        }
        u++;
    }
    return octetsImage;
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
        int biggerGroupSize;
        bool hasDoubleSurface = octet.hasDoubleSurface(config.PCADistThreshold,biggerGroupSize);
        if (hasDoubleSurface && biggerGroupSize<config.minOctetSize)
            return false;
        if (octet.partIds[1][1]==-1){
            octet.filterPos[1][1].u = v;
            octet.filterPos[1][1].v = u;
            if (config.useEuclideanCoordinates){
                sensorModel.getPoint(u,v,1.0,octet.partsPosEucl[1][1]);
            }
        }
        computeRelativePositions(octet, 1);
        octet.isBackground=false;
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
    if (octet.partIds[1][1]==-1){
        int depthNo=0;
        ViewDependentPart part;
        part.partIds = octet.partIds;
        part.partsPosNorm = octet.partsPosNorm;
        ViewDependentPart::removeSecondSurface(part);
        Octet octetTmp(octet);
        octetTmp.partIds=part.partIds;
        for (int i=0;i<3;i++){//compute mean depth
            for (int j=0;j<3;j++){
                if (octetTmp.partIds[i][j]!=-1){
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
                if (octet.partIds[i][j]==-1){
                    octet.filterPos[i][j].u=(j-1)*config.filterSize*(2.0*layerNo-1.0);//(j-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].v=(j-1)*config.filterSize*(2.0*layerNo-1.0);//(i-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].depth=meanDepth;
                    if (config.useEuclideanCoordinates){
                        octet.partsPosEucl[i][j]-=octet.partsPosEucl[1][1];
                        //sensorModel.getPoint(octet.filterPos[i][j].u,octet.filterPos[i][j].v,octet.filterPos[i][j].depth,octet.partsPosEucl[i][j]);
                    }
                    octet.partsPosNorm[i][j].mean.block<3,1>(0,0)-=octet.partsPosNorm[1][1].mean.block<3,1>(0,0);//should be boxplus
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
            if (responseImg[u+i][v+j].first!=-1){
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
void NormalImageFilter::getOctets(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){
    octets.clear();
    OctetsImage octetsImage = octetsImages[0][categoryNo][objectNo][imageNo];
    OctetsImage nextLayerOctetsImg(octetsImage.size()/3, std::vector<std::shared_ptr<Octet>> (octetsImage.back().size()/3));
    int u=0;
    for (size_t i=1; i<octetsImage.size()-1;i=i+3){
        int v=0;
        for (size_t j=1; j<octetsImage[0].size()-1;j=j+3){
            Octet octet;
            if (!isBackground(octetsImage, (int)i, (int)j)){
                if (fillInOctet(octetsImage, dictionary, (int)i, (int)j, octet)>=config.minOctetSize){
                    int biggerGroupSize;
                    bool hasDoubleSurface = octet.hasDoubleSurface(config.PCADistThreshold,biggerGroupSize);
                    if (!(hasDoubleSurface && biggerGroupSize<config.minOctetSize)){
                        computeRelativePositions(octet,2);
                        octet.isBackground=false;
                        octets.push_back(octet);
                        nextLayerOctetsImg[u][v].reset(new Octet(octet));
                    }
                }
            }
            v++;
        }
        u++;
    }
    updateOctetsImage(1,categoryNo, objectNo, imageNo, nextLayerOctetsImg);
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
void NormalImageFilter::computePartsImage(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, int layerNo){
    OctetsImage octetsImage = octetsImages[layerNo][categoryNo][objectNo][imageNo];
    PartsImage partsImage (octetsImage.size(), std::vector<std::shared_ptr<ViewDependentPart>> (octetsImage.back().size()));
    int partsNo = 0;
    for (size_t i=0; i<octetsImage.size();i++){
        for (size_t j=0; j<octetsImage.back().size();j++){
            ViewDependentPart part;
            part.id=-1;
            if (octetsImage[i][j].get()!=nullptr){
                if (!octetsImage[i][j]->isBackground){
                    Mat34 offset;
                    int id = findId(dictionary,*(octetsImage[i][j]), offset);
                    part = dictionary[id];
                    part.offset = offset;
                    part.id = id;
                    part.realisationId = partRealisationsCounter;
                    partRealisationsCounter++;
                    part.layerId=layerNo+1;
                    part.location = octetsImage[i][j]->filterPos[1][1];
                    part.locationEucl = octetsImage[i][j]->partsPosEucl[1][1];
                    //part.locationEucl = octetsImage[i][j]->partsPosNorm[1][1].mean.block<3,1>(0,0);
                    part.gaussians[1][1].mean=Vec3(octetsImage[i][j]->filterPos[1][1].u, octetsImage[i][j]->filterPos[1][1].v, octetsImage[i][j]->filterPos[1][1].depth);
                    partsNo++;
                    partsImage[i][j].reset(new ViewDependentPart(part));
                }
            }
        }
    }
    updatePartsImages(categoryNo, objectNo, imageNo, layerNo, partsImage);
}

/// get last view dependent layer parts from the image
void NormalImageFilter::getLayerParts(int categoryNo, int objectNo, int imageNo, int layerNo, std::vector<ViewDependentPart>& parts) const{
    parts.clear();
    for (size_t i=0; i<partsImages[layerNo][categoryNo][objectNo][imageNo].size();i++){
        for (size_t j=0; j<partsImages[layerNo][categoryNo][objectNo][imageNo].back().size();j++){
            if (partsImages[layerNo][categoryNo][objectNo][imageNo][i][j].get()!=nullptr){
                if (!partsImages[layerNo][categoryNo][objectNo][imageNo][i][j]->isBackground()){
                    parts.push_back(*partsImages[layerNo][categoryNo][objectNo][imageNo][i][j]);
                }
            }
        }
    }
}

/// Fill in octet
int NormalImageFilter::fillInOctet(const OctetsImage& octetsImage, const ViewDependentPart::Seq& dictionary, int u, int v, Octet& octet) const{
    int elementsNo=0;
    for (int i=-1;i<2;i++){
        for (int j=-1;j<2;j++){
            Mat34 offset(Mat34::Identity());
            int id;
            if (octetsImage[u+i][v+j].get()!=nullptr)
                id = findId(dictionary, *octetsImage[u+i][v+j], offset);
            else
                id = -1;
            octet.offsets[i+1][j+1]=offset;
            octet.partIds[i+1][j+1]=id;
            if (id ==-1){
                octet.filterPos[i+1][j+1].u=(v+j)*(config.filterSize*3)+((config.filterSize*3)/2);//octetsImage[u+i][v+j].filterPos[1][1];
                octet.filterPos[i+1][j+1].v=(u+i)*(config.filterSize*3)+((config.filterSize*3)/2);
                if (config.useEuclideanCoordinates)
                    sensorModel.getPoint(octet.filterPos[i+1][j+1].u, octet.filterPos[i+1][j+1].u, 1.0, octet.partsPosEucl[i+1][j+1]);
                octet.filterPos[i+1][j+1].depth = octet.partsPosEucl[i+1][j+1](3);
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=octet.partsPosEucl[i+1][j+1];
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=Vec3(0,0,1);
            }
            else {
                elementsNo++;
                octet.filterPos[i+1][j+1]=octetsImage[u+i][v+j]->filterPos[1][1];
                octet.partsPosEucl[i+1][j+1]=octetsImage[u+i][v+j]->partsPosEucl[1][1];
                //if (config.useEuclideanCoordinates)
                    //sensorModel.getPoint(octet.filterPos[i+1][j+1].u, octet.filterPos[i+1][j+1].v, octet.filterPos[i+1][j+1].depth, octet.partsPosEucl[i+1][j+1]);
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=octet.partsPosEucl[i+1][j+1];
                /*std::cout << i << ", " << j << " " << octet.partsPosEucl[i+1][j+1].transpose() << " pos\n";
                std::cout << i << ", " << j << " " << octet.partsPosEucl[i+1][j+1].transpose() << " pos\n";
                getchar();*/
                octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=dictionary[id].partsPosNorm[1][1].mean.block<3,1>(3,0);
                //octet.partsPosNorm[i+1][j+1].mean.block<3,1>(0,0)=(offset*Vec4(octet.partsPosEucl[i+1][j+1](0),octet.partsPosEucl[i+1][j+1](1),octet.partsPosEucl[i+1][j+1](2),1.0)).block<3,1>(0,3);
                //octet.partsPosNorm[i+1][j+1].mean.block<3,1>(3,0)=offset.rotation()*dictionary[id].partsPosNorm[1][1].mean.block<3,1>(3,0);
            }
            if (octetsImage[u+i][v+j].get()!=nullptr)
                octet.responses[i+1][j+1]=octetsImage[u+i][v+j]->responses[1][1];
        }
    }
    return elementsNo;
}

/// determine id of the part using dictionary
int NormalImageFilter::findId(const ViewDependentPart::Seq& dictionary, const Octet& octet) const{
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
}

/// determine id of the part using dictionary
int NormalImageFilter::findId(const ViewDependentPart::Seq& dictionary, const Octet& octet, Mat34& offset) const{
    if (octet.isBackground)//background
        return -1;
    int id=0; int foundId(0);
    double minDist=std::numeric_limits<double>::max();
    ViewDependentPart partVD;
    partVD.partsPosNorm = octet.partsPosNorm;
    partVD.partIds = octet.partIds;
    Mat34 offsetTmp;
    for (auto & part : dictionary){
        double dist = ViewDependentPart::distanceInvariant(part, partVD, 3, offsetTmp);
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
void NormalImageFilter::getPartsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart, int layersNo){
    unsigned int octetCoords[2]={u/(config.filterSize*3),v/(config.filterSize*3)};
    if ((octetCoords[0]<octetsImages[0][categoryNo][objectNo][imageNo].size())&&(octetCoords[1]<octetsImages[0][categoryNo][objectNo][imageNo][0].size())){
        if (octetsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            Octet octet = *(octetsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]);
            ids.push_back(octet.partIds[(u/(config.filterSize))%3][(v/(config.filterSize))%3]);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part
    }
    if (octetCoords[0]<partsImages[0][categoryNo][objectNo][imageNo].size()&&octetCoords[1]<partsImages[0][categoryNo][objectNo][imageNo][0].size()){
        if (partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            lastVDpart = *partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]];
            if (!partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->isBackground())
                ids.push_back(partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->id);
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        lastVDpart=ViewDependentPart();
        ids.push_back(-2);
    }
    unsigned int octetCoords2nd[2]={u/(config.filterSize*3*3),v/(config.filterSize*3*3)};
    if ((octetCoords2nd[0]<partsImages[1][categoryNo][objectNo][imageNo].size())&&(octetCoords2nd[1]<partsImages[1][categoryNo][objectNo][imageNo][0].size())){
        if (partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
            lastVDpart = *(partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]);
            if (!lastVDpart.isBackground())
                ids.push_back(lastVDpart.id);
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
        lastVDpart.id = -2;
    }
}

/// get set of ids for the given input point
void NormalImageFilter::getRealisationsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart){
    unsigned int octetCoords[2]={u/(config.filterSize*3),v/(config.filterSize*3)};
    if ((octetCoords[0]<octetsImages[0][categoryNo][objectNo][imageNo].size())&&(octetCoords[1]<octetsImages[0][categoryNo][objectNo][imageNo][0].size())){
        if (octetsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr)
            ids.push_back(octetsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->realisationsIds[(u/(config.filterSize))%3][(v/(config.filterSize))%3]);
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part
    }
    if (octetCoords[0]<partsImages[0][categoryNo][objectNo][imageNo].size()&&octetCoords[1]<partsImages[0][categoryNo][objectNo][imageNo][0].size()){
        if (partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]].get()!=nullptr){
            lastVDpart = *partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]];
            if (!partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->isBackground()){
                ids.push_back(partsImages[0][categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]]->realisationId);
            }
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        lastVDpart=ViewDependentPart();
        ids.push_back(-2);
    }
    unsigned int octetCoords2nd[2]={u/(config.filterSize*3*3),v/(config.filterSize*3*3)};
    if ((octetCoords2nd[0]<octetsImages[1][categoryNo][objectNo][imageNo].size())&&(octetCoords2nd[1]<octetsImages[1][categoryNo][objectNo][imageNo][0].size())){
        if (partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]].get()!=nullptr){
            lastVDpart = *partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]];
            if (!partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->isBackground())
                ids.push_back(partsImages[1][categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]]->realisationId);
            else
                ids.push_back(-2);
        }
        else
            ids.push_back(-2);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (3rd layer)
        lastVDpart.id = -2;
    }
}

/// Insertion operator
std::ostream& operator<<(std::ostream& os, const NormalImageFilter& filter){
    filter.save2file(os);
    return os;
}

/// save to file
void NormalImageFilter::save2file(std::ostream& os) const{
   /* os << inputClouds.size() << " ";
    for (auto &category : inputClouds){
        os << category.size() << " ";
        for (auto &object : category){
            os << object.size() << " ";
            for (auto &cloud : object){
                os << cloud.size() << " ";
                for (auto &point : cloud){
                    os << point.position(0) << " " << point.position(1) << " " << point.position(2) << " ";
                    if (std::isnan(point.normal(0))||std::isnan(point.normal(1))||std::isnan(point.normal(2)))
                        os << -2 << " " << -2 << " " << -2 << " ";
                    else
                        os << point.normal(0) << " " << point.normal(1) << " " << point.normal(2) << " ";
                    os << point.u << " " << point.v << " ";
                }
            }
        }
    }*/
    os << octetsImages.size() << " ";
    for (auto &layer : octetsImages){
        os << layer.size() << " ";
        for (auto &category : layer){
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
    os << partsImages.size() << " ";
    for (auto &layer : partsImages){
        os << layer.size() << " ";
        for (auto &category : layer){
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
    os << "\n";
}

// Extraction operator
std::istream& operator>>(std::istream& is, NormalImageFilter& filter){
    filter.loadFromfile(is);
    return is;
}

/// load from file
void NormalImageFilter::loadFromfile(std::istream& is){
    // read categories no
    int categoriesNo;
    /*is >> categoriesNo;
    inputClouds.clear();
    inputClouds.resize(categoriesNo);
    for (int catNo = 0; catNo<categoriesNo; catNo++){
        int objectsNo;
        is >> objectsNo;
        inputClouds[catNo].resize(objectsNo);
        for (int objNo=0;objNo<objectsNo; objNo++){
            int cloudsNo;
            is >> cloudsNo;
            inputClouds[catNo][objNo].resize(cloudsNo);
            for (int cloudNo=0;cloudNo<cloudsNo;cloudNo++){
                int pointsNo;
                is >> pointsNo;
                PointCloudUV cloud;
                //cloud.reserve(pointsNo);
                for (int pointNo=0;pointNo<pointsNo;pointNo++){
                    hop3d::PointNormalUV point;
                    is >> point.position(0) >> point.position(1) >> point.position(2);
                    is >> point.normal(0) >> point.normal(1) >> point.normal(2);
                    if (point.normal(0)==-2){
                        point.normal = Vec3(NAN,NAN,NAN);
                    }
                    is >> point.u >> point.v;
                    cloud.push_back(point);
                }
                inputClouds[catNo][objNo][cloudNo] = cloud;
            }
        }
    }*/
    int layersNo;
    is >> layersNo;
    octetsImages.clear();
    octetsImages.resize(layersNo);
    for (int layNo = 0; layNo<layersNo; layNo++){
        is >> categoriesNo;
        octetsImages[layNo].resize(categoriesNo);
        for (int catNo = 0; catNo<categoriesNo; catNo++){
            int objectsNo;
            is >> objectsNo;
            octetsImages[layNo][catNo].resize(objectsNo);
            for (int objNo=0;objNo<objectsNo; objNo++){
                int imgsNo;
                is >> imgsNo;
                octetsImages[layNo][catNo][objNo].resize(imgsNo);
                for (int imgNo=0;imgNo<imgsNo;imgNo++){
                    int rowsNo;
                    is >> rowsNo;
                    octetsImages[layNo][catNo][objNo][imgNo].resize(rowsNo);
                    for (int rowNo=0;rowNo<rowsNo;rowNo++){
                        int octetsNo;
                        is >> octetsNo;
                        octetsImages[layNo][catNo][objNo][imgNo][rowNo].resize(octetsNo);
                        for (int octetNo=0;octetNo<octetsNo;octetNo++){
                            int isOctet;
                            is >> isOctet;
                            if (isOctet){
                                Octet octet;
                                is >> octet;
                                octetsImages[layNo][catNo][objNo][imgNo][rowNo][octetNo].reset(new Octet(octet));
                            }
                        }
                    }
                }
            }
        }
    }

    is >> layersNo;
    partsImages.clear();
    partsImages.resize(layersNo);
    for (int layNo = 0; layNo<layersNo; layNo++){
        int catsNo;
        is >> catsNo;
        partsImages[layNo].resize(catsNo);
        for (int catNo = 0; catNo<catsNo; catNo++){
            int objectsNo;
            is >> objectsNo;
            partsImages[layNo][catNo].resize(objectsNo);
            for (int objNo=0;objNo<objectsNo; objNo++){
                int imgsNo;
                is >> imgsNo;
                partsImages[layNo][catNo][objNo].resize(imgsNo);
                for (int imgNo=0;imgNo<imgsNo;imgNo++){
                    int rowsNo;
                    is >> rowsNo;
                    partsImages[layNo][catNo][objNo][imgNo].resize(rowsNo);
                    for (int rowNo=0;rowNo<rowsNo;rowNo++){
                        int partsNo;
                        is >> partsNo;
                        partsImages[layNo][catNo][objNo][imgNo][rowNo].resize(partsNo);
                        for (int partNo=0;partNo<partsNo;partNo++){
                            int isPart;
                            is >> isPart;
                            if (isPart){
                                ViewDependentPart part;
                                is >> part;
                                partsImages[layNo][catNo][objNo][imgNo][rowNo][partNo].reset(new ViewDependentPart(part));
                            }
                        }
                    }
                }
            }
        }
    }
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(void) {
    filterNorm.reset(new NormalImageFilter());
    return filterNorm.get();
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(std::string config, std::string sensorConfig) {
    filterNorm.reset(new NormalImageFilter(config, sensorConfig));
    return filterNorm.get();
}
