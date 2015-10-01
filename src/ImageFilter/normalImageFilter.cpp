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
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("scalingToMeters", &scalingToMeters);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("backgroundThreshold", &backgroundThreshold);

    if (verbose>0){
        std::cout << "Load normal filter parameters...\n";
        std::cout << "Rings no.: " << ringsNo << "\n";
        std::cout<< "Filter size in pixels: " << filterSize << std::endl;
        std::cout<< "Verbose: " << verbose << std::endl;
        std::cout<< "Scaling of raw int16 depth values into meters: " << scalingToMeters << std::endl;
    }

 }

const std::string& NormalImageFilter::getName() const {
    return name;
}

///compute set of octets from set of the depth images
void NormalImageFilter::computeOctets(const cv::Mat& depthImage, hop3d::Octet::Seq& octets){
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
    std::vector< std::vector<Response> > responseImage(depthImage.rows, std::vector<Response>(depthImage.cols,std::make_pair<int, double>(-1,-1.0)));
    for (int i=0;i<depthImage.rows;i++){
        for (int j=0;j<depthImage.cols;j++){
            sensorModel.getPoint(i, j, depthImage.at<uint16_t>(i,j)*config.scalingToMeters, cloudOrd[i][j].position);
        }
    }
    cv::Mat idsImage(depthImage.rows,depthImage.cols, cv::DataType<uchar>::type,cv::Scalar(0));
    for (int i=config.filterSize/2;i<depthImage.rows-(config.filterSize/2);i++){
        for (int j=config.filterSize/2;j<depthImage.cols-(config.filterSize/2);j++){
            if (!std::isnan(cloudOrd[i][j].position(2))){
                computeNormal(i, j, cloudOrd);
                //compute id
                if (!std::isnan(cloudOrd[i][j].normal(2))){
                    responseImage[i][j].first = toId(cloudOrd[i][j].normal);
                    //compute response (dot product)
                    responseImage[i][j].second = cloudOrd[i][j].normal.adjoint()*filters[responseImage[i][j].first].normal;
                    //std::cout << "normal " << cloudOrd[i][j].normal.transpose() << "\n";
                    //std::cout << "filter normal " << filters[responseImage[i][j].first].normal.transpose() << "\n";
                    //std::cout << responseImage[i][j].first << " -> " << responseImage[i][j].second << "\n";
                    //getchar();
                    if (config.verbose)
                        idsImage.at<uchar>(i,j)=(uchar)(toId(cloudOrd[i][j].normal)*2);
                }
            }
        }
    }
    if (config.verbose>0){
        std::chrono::steady_clock::time_point end=std::chrono::steady_clock::now();
        std::cout << "Octets extraction takes " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms" << std::endl;
    }
    if (config.verbose){
        hop3d::ImagesDisplay displayer;
        namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
        imshow( "Display window", idsImage );
        cv::waitKey(0);
    }
}

/// compute set of octets from set of the depth image
void NormalImageFilter::getOctets(Octet::Seq& octets){
    std::cout << octets.size();
}

/// compute set of octets from set of the ids image
void NormalImageFilter::getOctets(const ViewDependentPart::Seq& dictionary, Octet::Seq& octets){
    std::cout << dictionary.size();
    std::cout << octets.size();
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
    pointNormal.normal = Vec3(std::real(es.eigenvectors()(0,min)), std::real(es.eigenvectors()(1,min)), std::real(es.eigenvectors()(2,min)));
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
void NormalImageFilter::normalizeVector(Vec3& normal) const {
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
