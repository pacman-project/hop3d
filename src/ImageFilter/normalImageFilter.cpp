#include "ImageFilter/normalImageFilter.h"
#include <chrono>

using namespace hop3d;

/// A single instance of Features Map
NormalImageFilter::Ptr filterNorm;

NormalImageFilter::NormalImageFilter(void) : ImageFilter("Depth image filter using normals", FILTER_NORMAL) {
    generateFilters();
}

/// Construction
NormalImageFilter::NormalImageFilter(std::string config, std::string sensorConfig) :
        ImageFilter("Depth image filter using normals", FILTER_NORMAL), config(config), sensorModel(sensorConfig) {
    generateFilters();
}

/// Destruction
NormalImageFilter::~NormalImageFilter(void) {
}

///config class constructor
NormalImageFilter::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load depth filter (normal) config file.\n";
    tinyxml2::XMLElement * model = config.FirstChildElement( "NormalFilterer" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("ringsNo", &ringsNo);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("filterSize", &filterSize);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("nonMaximumSupressionGroup", &nonMaximumSupressionGroup);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("minOctetSize", &minOctetSize);

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

/// compute median
uint16_t NormalImageFilter::median(const cv::Mat& inputImg, int u, int v, int kernelSize){
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
void NormalImageFilter::medianFilter(const cv::Mat& inputImg, cv::Mat& outputImg, int kernelSize){
    for (int i=0;i<inputImg.rows;i++){
        for (int j=0;j<inputImg.cols;j++){
            outputImg.at<uint16_t>(i,j)=median(inputImg, i, j, kernelSize);
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
    /// response: id, response value
    std::vector< std::vector<Response> > responseImage(depthImage.rows, std::vector<Response>(depthImage.cols,std::make_pair<int, double>(-1,-1.0)));
    double scale = 1/sensorModel.config.depthImageScale;
    if (config.verbose>1)
        imshow( "Input image", depthImage );
    cv::Mat filteredImg = depthImage.clone();
    if (config.useMedianFilter){
        if (config.kernelSize<6)
            cv::medianBlur(depthImage, filteredImg, config.kernelSize);
        else {
            medianFilter(depthImage, filteredImg, config.kernelSize);
        }
    }
    for (int i=0;i<filteredImg.rows;i++){
        for (int j=0;j<filteredImg.cols;j++){
            sensorModel.getPoint(i, j, filteredImg.at<uint16_t>(i,j)*scale, cloudOrd[i][j].position);
        }
    }
    cv::Mat idsImage(filteredImg.rows,filteredImg.cols, cv::DataType<int>::type,cv::Scalar(0));
    for (int i=config.filterSize/2;i<filteredImg.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<filteredImg.cols-(config.filterSize/2);j++){
			if (!std::isnan(double(cloudOrd[i][j].position(2)))){
                computeNormal(i, j, cloudOrd);
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
    updateOctetsImages1stLayer(categoryNo, objectNo, imageNo, octetsImage);

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
void NormalImageFilter::updateOctetsImages1stLayer(int categoryNo, int objectNo, int imageNo, const OctetsImage& octetsImage){
    if((int)octetsImages1stLayer.size()<=categoryNo){
        octetsImages1stLayer.resize(categoryNo+1);
    }
    if ((int)octetsImages1stLayer[categoryNo].size()<=objectNo){
        octetsImages1stLayer[categoryNo].resize(objectNo+1);
    }
    if ((int)octetsImages1stLayer[categoryNo][objectNo].size()<=imageNo){
        octetsImages1stLayer[categoryNo][objectNo].resize(imageNo+1);
    }
    octetsImages1stLayer[categoryNo][objectNo][imageNo] = octetsImage;
}

/// returs filter ids and their position on the image
void NormalImageFilter::getResponseFilters(int categoryNo, int objectNo, int imageNo, std::vector<PartCoords>& partCoords) const {
    partCoords.clear();
    for (auto& row : octetsImages1stLayer[categoryNo][objectNo][imageNo]){
        for (auto& octet : row){
            if (!octet.isBackground){
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        if (octet.partIds[i][j]!=-1){
                            if (i==1&&j==1){
                                PartCoords fcoords(octet.partIds[i][j], octet.filterPos[i][j]);
                                partCoords.push_back(fcoords);
                            }
                            else{
                                PartCoords fcoords(octet.partIds[i][j], octet.filterPos[1][1]+octet.filterPos[i][j]);
                                partCoords.push_back(fcoords);
                            }
                        }
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
        for (auto& row : octetsImages2ndLayer[categoryNo][objectNo][imageNo]){
            for (auto& octet : row){
                if (!octet.isBackground){
                    for (int i=0;i<3;i++){
                        for (int j=0;j<3;j++){
                            if (octet.partIds[i][j]!=-1){
                                if (i==1&&j==1){
                                    PartCoords fcoords(octet.partIds[i][j], octet.filterPos[i][j]);
                                    partCoords.push_back(fcoords);
                                }
                                else{
                                    PartCoords fcoords(octet.partIds[i][j], octet.filterPos[1][1]+octet.filterPos[i][j]);
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

/// update structure which holds octets images
void NormalImageFilter::updateOctetsImages2ndLayer(int categoryNo, int objectNo, int imageNo, const OctetsImage& octetsImage){
    if((int)octetsImages2ndLayer.size()<=categoryNo){
        octetsImages2ndLayer.resize(categoryNo+1);
    }
    if ((int)octetsImages2ndLayer[categoryNo].size()<=objectNo){
        octetsImages2ndLayer[categoryNo].resize(objectNo+1);
    }
    if ((int)octetsImages2ndLayer[categoryNo][objectNo].size()<=imageNo){
        octetsImages2ndLayer[categoryNo][objectNo].resize(imageNo+1);
    }
    octetsImages2ndLayer[categoryNo][objectNo][imageNo] = octetsImage;
}

/// update structure which holds parts images
void NormalImageFilter::updatePartsImages(int categoryNo, int objectNo, int imageNo, const PartsImage& partsImage){
    if((int)partsImages.size()<=categoryNo){
        partsImages.resize(categoryNo+1);
    }
    if ((int)partsImages[categoryNo].size()<=objectNo){
        partsImages[categoryNo].resize(objectNo+1);
    }
    if ((int)partsImages[categoryNo][objectNo].size()<=imageNo){
        partsImages[categoryNo][objectNo].resize(imageNo+1);
    }
    partsImages[categoryNo][objectNo][imageNo] = partsImage;
}

///extract octets from response image
NormalImageFilter::OctetsImage NormalImageFilter::extractOctets(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, hop3d::Octet::Seq& octets){
    int u=0;
    OctetsImage octetsImage(responseImg.size()/(config.filterSize*3), std::vector<Octet> (responseImg[0].size()/(config.filterSize*3)));
    for (size_t i=config.filterSize+config.filterSize/2;i<responseImg.size()-config.filterSize-(config.filterSize/2);i=i+3*config.filterSize){
        int v=0;
        for (size_t j=config.filterSize+config.filterSize/2;j<responseImg[0].size()-config.filterSize-(config.filterSize/2);j=j+3*config.filterSize){
            Octet octet;
            if (computeOctet(responseImg, cloudOrd, int(i), int(j), octet)){
                octetsImage[u][v]=octet;
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
        }
    }
    if (elementsNo>config.minOctetSize){//set relative position for octets
        if (octet.partIds[1][1]==-1){
            octet.filterPos[1][1].u = v;
            octet.filterPos[1][1].v = u;
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
    double min=std::numeric_limits<double>::max();
    double max=std::numeric_limits<double>::min();
    //double globU=0, globV=0;
    if (octet.partIds[1][1]==-1){
        int depthNo=0;
        for (int i=0;i<3;i++){//compute mean depth
            for (int j=0;j<3;j++){
                if (octet.partIds[i][j]!=-1){
                    meanDepth+= octet.filterPos[i][j].depth;
                    if (min>octet.filterPos[i][j].depth){
                        min = octet.filterPos[i][j].depth;
                        //globU = octet.filterPos[i][j].u;
                        //globV = octet.filterPos[i][j].v;
                    }
                    if (max<octet.filterPos[i][j].depth){
                        max = octet.filterPos[i][j].depth;
                    }
                    depthNo++;
                }
            }
        }
        meanDepth /= double(depthNo);
        octet.filterPos[1][1].depth = meanDepth;
        //octet.filterPos[1][1].depth = min;
        //octet.filterPos[1][1].u = globU;
        //octet.filterPos[1][1].v = globV;
    }
    for (int i=0;i<3;i++){//for neighbouring blocks
        for (int j=0;j<3;j++){
            if (!((i==1)&&(j==1))){
                if (octet.partIds[i][j]==-1){
                    octet.filterPos[i][j].u=(j-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].v=(i-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].depth=meanDepth;
                }
                else {
                    octet.filterPos[i][j].u-=octet.filterPos[1][1].u;
                    octet.filterPos[i][j].v-=octet.filterPos[1][1].v;
                    octet.filterPos[i][j].depth-=octet.filterPos[1][1].depth;
                }
            }
        }
    }
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
                isBackground = false;
            }
        }
    }
    return !isBackground;// succes if not background
}

/// compute max response for the most numerous group in the window
bool NormalImageFilter::findMaxGroupResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const{
    octet.responses[idx][idy]=-1;    octet.partIds[idx][idy]=-1;
    std::map<int,int> occurencesMap;
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
        for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
            for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
                if ((responseImg[u+i][v+j].second>octet.responses[idx][idy])&&(std::find(maxIds.begin(), maxIds.end(), responseImg[u+i][v+j].first) != maxIds.end())){
                    octet.responses[idx][idy] = responseImg[u+i][v+j].second;
                    octet.partIds[idx][idy] = responseImg[u+i][v+j].first;
                    ImageCoordsDepth coord;
                    coord.u = v+j; coord.v = u+i; coord.depth = cloudOrd[u+i][v+j].position(2);
                    octet.filterPos[idx][idy] = coord;
                    isBackground = false;
                }
            }
        }
    }
    return !isBackground;// succes if not background
}

/// compute set of octets from set of the ids image
void NormalImageFilter::getOctets(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){
    octets.clear();
    OctetsImage octetsImage = octetsImages1stLayer[categoryNo][objectNo][imageNo];
    OctetsImage nextLayerOctetsImg(octetsImage.size()/3, std::vector<Octet> (octetsImage.back().size()/3));
    int u=0;
    for (size_t i=1; i<octetsImage.size()-1;i=i+3){
        int v=0;
        for (size_t j=1; j<octetsImage[0].size()-1;j=j+3){
            Octet octet;
            if (!isBackground(octetsImage, (int)i, (int)j)){
                fillInOctet(octetsImage, dictionary, (int)i, (int)j, octet);
                computeRelativePositions(octet,2);
                octet.isBackground=false;
                octets.push_back(octet);
                nextLayerOctetsImg[u][v]=octet;
            }
            v++;
        }
        u++;
    }
    updateOctetsImages2ndLayer(categoryNo, objectNo, imageNo, nextLayerOctetsImg);
}

/// check if octet is background
bool NormalImageFilter::isBackground(OctetsImage& octetsImage, int u, int v) const{
    for (int i=-1;i<2;i++){
        for (int j=-1;j<2;j++){
            if (!octetsImage[i+u][j+v].isBackground){
                return false;
            }
        }
    }
    return true;
}

/// define 2rd layer octet images using selected words from third layer
void NormalImageFilter::computeImages3rdLayer(int categoryNo, int objectNo, int imageNo, const ViewDependentPart::Seq& dictionary){
    PartsImage partsImage (octetsImages2ndLayer[categoryNo][objectNo][imageNo].size(), std::vector<ViewDependentPart> (octetsImages2ndLayer[categoryNo][objectNo][imageNo].back().size()));
    OctetsImage octetsImage = octetsImages2ndLayer[categoryNo][objectNo][imageNo];
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
                int id = findId(dictionary,octetsImage[i][j]);
                part = dictionary[id];
                part.id = id;
                part.layerId=3;
                //std::cout << "found id : " << findId(dictionary,octetsImage[i][j]) << " new id " << dictionary[findId(dictionary,octetsImage[i][j])].id << "\n";
                //std::cout << "parts no " << partsNo << "\n";
                part.location = octetsImage[i][j].filterPos[1][1];
                part.gaussians[1][1].mean=Vec3(octetsImage[i][j].filterPos[1][1].u, octetsImage[i][j].filterPos[1][1].v, octetsImage[i][j].filterPos[1][1].depth);
                partsNo++;
            }
            partsImage[i][j]=part;
        }
        //std::cout << "\n";
    }
    updatePartsImages(categoryNo, objectNo, imageNo, partsImage);
}

/// get last view dependent layer parts from the image
void NormalImageFilter::getLastVDLayerParts(int categoryNo, int objectNo, int imageNo, std::vector<ViewDependentPart>& parts) const{
    parts.clear();
    PartsImage partsImage = partsImages[categoryNo][objectNo][imageNo];
    for (size_t i=0; i<partsImage.size();i++){
        for (size_t j=0; j<partsImage.back().size();j++){
            if (!partsImage[i][j].isBackground()){
                parts.push_back(partsImage[i][j]);
            }
        }
    }
}

/// Fill in octet
void NormalImageFilter::fillInOctet(const OctetsImage& octetsImage, const ViewDependentPart::Seq& dictionary, int u, int v, Octet& octet) const{
    for (int i=-1; i<2;i++){
        for (int j=-1;j<2;j++){
            int id = findId(dictionary, octetsImage[u+i][v+j]);
            octet.partIds[i+1][j+1]=id;
            if (id ==-1){
                octet.filterPos[i+1][j+1].u=(v+j)*(config.filterSize*3)+((config.filterSize*3)/2);//octetsImage[u+i][v+j].filterPos[1][1];
                octet.filterPos[i+1][j+1].v=(u+i)*(config.filterSize*3)+((config.filterSize*3)/2);
            }
            else
                octet.filterPos[i+1][j+1]=octetsImage[u+i][v+j].filterPos[1][1];
            octet.responses[i+1][j+1]=octetsImage[u+i][v+j].responses[1][1];
        }
    }
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
void NormalImageFilter::computeNormal(int u, int v, std::vector< std::vector<hop3d::PointNormal> >& cloudOrd){
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
    if ((double(clusters[smaller].size())/double(clusters[bigger].size()))<0.75)
        return false;

    if (fabs(dist/width)>config.PCARelDistClusters){
        int clusterNo = bigger;//(minDist[1]>minDist[0]) ? 0 : 1;
        pointGroup.clear();
        for (auto& id : clusters[clusterNo])
            pointGroup.push_back(points[id]);
        return true;
    }
    return false;
}

/// Compute normal vector using PCA
void NormalImageFilter::normalPCA(std::vector<hop3d::PointNormal>& points, hop3d::PointNormal& pointNormal){
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
void NormalImageFilter::getPartsIds(int categoryNo, int objectNo, int imageNo, unsigned int u, unsigned int v, std::vector<int>& ids, ViewDependentPart& lastVDpart){
    // get filter id
    //std::cout << "u v " << u << ", " << v << "\n";
    /*for (int i=0;i<octetsImages2ndLayer[categoryNo][objectNo][imageNo].size();i++){
        for (int j=0;j<octetsImages2ndLayer[categoryNo][objectNo][imageNo][0].size();j++){
            if (!octetsImages2ndLayer[categoryNo][objectNo][imageNo][i][j].isBackground)
                octetsImages2ndLayer[categoryNo][objectNo][imageNo][i][j].print();
        }
    }
    getchar();*/
    unsigned int octetCoords[2]={u/(config.filterSize*3),v/(config.filterSize*3)};
//    std::cout << "coord 1st " << octetCoords[0] << ", " << octetCoords[1] << "\n";
//    std::cout << "1st image size " << octetsImages1stLayer[categoryNo][objectNo][imageNo].size() << "x" << octetsImages1stLayer[categoryNo][objectNo][imageNo][0].size() << "\n";
    /*for (int i=0;i<octetsImages1stLayer[categoryNo][objectNo][imageNo].size();i++){
        for (int j=0;j<octetsImages1stLayer[categoryNo][objectNo][imageNo][0].size();j++){
            if (octetsImages1stLayer[categoryNo][objectNo][imageNo][i][j].partIds[1][1]!=-1){
                std::cout << "ij: " << i << " " << j << " mmm " << i*config.filterSize*3+(config.filterSize*3/2) << " mmmj " << j*config.filterSize*3+(config.filterSize*3/2) << "\n";
            }
        }
    }*/
    if ((octetCoords[0]<octetsImages1stLayer[categoryNo][objectNo][imageNo].size())&&(octetCoords[1]<octetsImages1stLayer[categoryNo][objectNo][imageNo][0].size())){
        Octet octet = octetsImages1stLayer[categoryNo][objectNo][imageNo][octetCoords[0]][octetCoords[1]];
  //      std::cout << "coord 1st id " << (v/(config.filterSize))%3 << ", " << (u/(config.filterSize))%3 << " -> " << octet.partIds[(v/(config.filterSize))%3][(u/(config.filterSize))%3] << "\n";
        ids.push_back(octet.partIds[(u/(config.filterSize))%3][(v/(config.filterSize))%3]);
    }
    else{
        ids.push_back(-2); //point wasn't used to create part
    }
    unsigned int octetCoords2nd[2]={u/(config.filterSize*3*3),v/(config.filterSize*3*3)};
    //std::cout << "coord 2st " << octetCoords2nd[0] << ", " << octetCoords2nd[1] << "\n";
    //std::cout << "2st image size " << octetsImages2ndLayer[categoryNo][objectNo][imageNo].size() << "x" << octetsImages2ndLayer[categoryNo][objectNo][imageNo][0].size() << "\n";
    if ((octetCoords2nd[0]<octetsImages2ndLayer[categoryNo][objectNo][imageNo].size())&&(octetCoords2nd[1]<octetsImages2ndLayer[categoryNo][objectNo][imageNo][0].size())){
        Octet octet2nd = octetsImages2ndLayer[categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]];
      //  std::cout << "coord 2st id " << (v/(config.filterSize*3))%3 << ", " << (u/(config.filterSize*3))%3 << " -> " << octet2nd.partIds[(v/(config.filterSize*3))%3][(u/(config.filterSize*3))%3] << "\n";
        ids.push_back(octet2nd.partIds[(u/(config.filterSize*3))%3][(v/(config.filterSize*3))%3]);
//        std::cout << " octet 2nd:\n";
        /*for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                std::cout << octet2nd.partIds[i][j] << ", ";
            }
            std::cout << "\n";
        }*/
        lastVDpart = partsImages[categoryNo][objectNo][imageNo][octetCoords2nd[0]][octetCoords2nd[1]];
        ids.push_back(lastVDpart.id);
        /*std::cout << "part.id " << part.id << "\n";
        std::cout << " l2 id " << part.partIds[(v/(config.filterSize*3))%3][(u/(config.filterSize*3))%3] << "\n";
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                std::cout << part.partIds[i][j] << ", ";
            }
            std::cout << "\n";
        }*/
        //std::cout << "coords " << part.location.u << " " << part.location.v << "\n";
    }
    else{
        ids.push_back(-2); //point wasn't used to create part (2nd layer)
        ids.push_back(-2); //point wasn't used to create part (3rd layer)
        lastVDpart.id = -2;
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
