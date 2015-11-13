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
    //model->FirstChildElement( "parameters" )->QueryDoubleAttribute("scalingToMeters", &scalingToMeters);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("backgroundThreshold", &backgroundThreshold);

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
    cv::Mat idsImage(filteredImg.rows,filteredImg.cols, cv::DataType<uchar>::type,cv::Scalar(0));
    for (int i=config.filterSize/2;i<filteredImg.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<filteredImg.cols-(config.filterSize/2);j++){
            if (!std::isnan(cloudOrd[i][j].position(2))){
                computeNormal(i, j, cloudOrd);
                //compute id
                if (!std::isnan(cloudOrd[i][j].normal(2))){
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
            computeOctet(responseImg, cloudOrd, int(i), int(j), octet);
            if (octet.partIds[1][1]!=-1){
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
void NormalImageFilter::computeOctet(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u,  int v, Octet& octet) const{
    for (int i=-1;i<2;i++){//for neighbouring blocks in octet
        for (int j=-1;j<2;j++){
            findMaxResponse(responseImg, cloudOrd, u+i*config.filterSize, v+j*config.filterSize, octet, i+1, j+1);
        }
    }
    if (octet.partIds[1][1]!=-1){//set relative position for octets
        computeRelativePositions(octet, 1);
    }
}

//set relative position for octets
void NormalImageFilter::computeRelativePositions(Octet& octet, int layerNo) const{
    for (int i=0;i<3;i++){//for neighbouring blocks
        for (int j=0;j<3;j++){
            if (!((i==1)&&(j==1))){
                if (octet.partIds[i][j]==-1){
                    octet.filterPos[i][j].u=(j-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].v=(i-1)*layerNo*(config.filterSize+config.filterSize/2.0);
                    octet.filterPos[i][j].depth=0;
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

/// compute max response in vindow
void NormalImageFilter::findMaxResponse(const std::vector< std::vector<Response> >& responseImg, const std::vector< std::vector<hop3d::PointNormal> >& cloudOrd, int u, int v, Octet& octet, int idx, int idy) const{
    octet.responses[idx][idy]=-1;    octet.partIds[idx][idy]=-1;
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            if (responseImg[u+i][v+j].second>octet.responses[idx][idy]){
                octet.responses[idx][idy] = responseImg[u+i][v+j].second;
                octet.partIds[idx][idy] = responseImg[u+i][v+j].first;
                ImageCoordsDepth coord;
                coord.u = v+j; coord.v = u+i; coord.depth = cloudOrd[u+i][v+j].position(2);
                octet.filterPos[idx][idy] = coord;
            }
        }
    }
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
            if (octetsImage[i][j].partIds[1][1]!=-1){
                fillInOctet(octetsImage, dictionary, (int)i, (int)j, octet);
                computeRelativePositions(octet,2);
                octets.push_back(octet);
                nextLayerOctetsImg[u][v]=octet;
            }
            v++;
        }
        u++;
    }
    updateOctetsImages2ndLayer(categoryNo, objectNo, imageNo, nextLayerOctetsImg);
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
            if (octetsImage[i][j].partIds[1][1]!=-1){
                //std::cout << dictionary.size() << "\n";
                //std::cout << "findId(dictionary,octetsImage[i][j]) " << findId(dictionary,octetsImage[i][j]) << "\n";
                int id = findId(dictionary,octetsImage[i][j]);
                part = dictionary[id];
                part.id = id;
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
            if (partsImage[i][j].id!=-1){
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
            octet.filterPos[i+1][j+1]=octetsImage[u+i][v+j].filterPos[1][1];
            octet.responses[i+1][j+1]=octetsImage[u+i][v+j].responses[1][1];
        }
    }
}

/// determine id of the part using dictionary
int NormalImageFilter::findId(const ViewDependentPart::Seq& dictionary, const Octet& octet) const{
    if (octet.partIds[1][1]==-1)//background
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
    for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
        for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
            if (!std::isnan(cloudOrd[u+i][v+j].position(2))){
                points.push_back(cloudOrd[u+i][v+j]);
               // std::cout << "point " << cloudOrd[u+i][v+j].position << "\n";
            }
        }
    }
    if (points.size()>size_t(config.backgroundThreshold))
        normalPCA(points, cloudOrd[u][v]);
    else
        cloudOrd[u][v].normal = Vec3(NAN,NAN,NAN);
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
    else if (std::real(es.eigenvalues()(2))<std::real(es.eigenvalues()(1)))
        min=2;
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
        for (int i=-config.filterSize/2;i<1+config.filterSize/2;i++){
            int v=0;
            for (int j=-config.filterSize/2;j<1+config.filterSize/2;j++){
                Mat34 point(Mat34::Identity());
                point(0,3) = double(i); point(1,3) = double(j);
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
    normalizeVector(y);
    x = y.cross(normal);
    normalizeVector(x);
    Mat33 R;
    R.block(0,0,3,1) = x;
    R.block(0,1,3,1) = y;
    R.block(0,2,3,1) = normal;
    return R;
}

/// normalize vector
void NormalImageFilter::normalizeVector(Vec3& normal){
    double norm = normal.norm();
    normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(void) {
    filterNorm.reset(new NormalImageFilter());
    return filterNorm.get();
}

hop3d::ImageFilter* hop3d::createNormalImageFilter(std::string config, std::string sensorConfig) {
    filterNorm.reset(new NormalImageFilter(config, sensorConfig));
    return filterNorm.get();
}
