#include "../include/hop3d/Dataset/datasetPacman.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"


/// map idx to viewig angle
std::map<int,int> idx2Angle;
std::vector<int> angles={-164,-164,-135,-135,-135,-135,-106,-106,-74,-74,-45,-45,-45,-45,-16,-16,16,16,45,45,45,45,74,74,106,106,135,135,135,135,164,164,-16,16,-68,-35,35,68,-16,16,-16,16,-68,-35,35,68,-16,16,-16,16,-68,-35,35,68,-16,16,-16,16,-68,-35,35,68,-16,16};

using namespace hop3d;

/// A single instance of PacmanDataset
//PacmanDataset::Ptr datasetPacman;

PacmanDataset::PacmanDataset(void) : Dataset("Pacman dataset", DATASET_PACMAN) {
    idx2Angle.clear();
    for (size_t id=0; id<angles.size();id++){
        idx2Angle.insert(std::make_pair(id+1,angles[id]));
    }
}

/// Construction
PacmanDataset::PacmanDataset(std::string config, std::string configSensor) :
        Dataset("Pacman dataset", DATASET_PACMAN), config(config), sensorModel(configSensor) {
    idx2Angle.clear();
    for (size_t id=0; id<angles.size();id++){
        idx2Angle.insert(std::make_pair(id+1,angles[id]));
    }
}

/// Destruction
PacmanDataset::~PacmanDataset(void) {
}

///config class constructor
PacmanDataset::Config::Config(std::string configFilename){
    idx2Angle.clear();
    for (size_t id=0; id<angles.size();id++){
        idx2Angle.insert(std::make_pair(id+1,angles[id]));
    }
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        throw std::runtime_error("unable to load Pacman dataset config file: " + filename);
    size_t found = configFilename.find_last_of("/\\");
    std::string prefix = configFilename.substr(0,found+1);

    tinyxml2::XMLElement * model = config.FirstChildElement( "Dataset" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    int categoriesNo;
    model->FirstChildElement( "parameters" )->QueryIntAttribute("categoriesNo", &categoriesNo);
    dataset.categories.resize(categoriesNo);
    type = model->FirstChildElement( "parameters" )->Attribute( "type" );
    path = prefix+model->FirstChildElement( "parameters" )->Attribute( "path" );
    for (int categoryNo=0;categoryNo<categoriesNo; categoryNo++){
        std::stringstream categoryName;
        categoryName << "Category" << categoryNo;
        dataset.categories[categoryNo].name = model->FirstChildElement( categoryName.str().c_str() )->Attribute( "name" );
        if (verbose>1)
            std::cout << dataset.categories[categoryNo].name <<"\n";
        int objectsNo;
        model->FirstChildElement( categoryName.str().c_str() )->QueryIntAttribute("objectsNo", &objectsNo);
        dataset.categories[categoryNo].objects.resize(objectsNo);
        for (int objectNo=0;objectNo<objectsNo; objectNo++){
            std::stringstream objectName;
            objectName << "Object" << objectNo;
            dataset.categories[categoryNo].objects[objectNo].name = model->FirstChildElement( categoryName.str().c_str() )->FirstChildElement( objectName.str().c_str() )->Attribute( "name" );
            categories.emplace(std::make_pair(dataset.categories[categoryNo].objects[objectNo].name, categoryNo));
            objects.insert(std::make_pair(dataset.categories[categoryNo].objects[objectNo].name,objectNo));
            if (verbose>1)
                std::cout << dataset.categories[categoryNo].objects[objectNo].name <<"\n";
            int imagesNo;
            model->FirstChildElement( categoryName.str().c_str() )->FirstChildElement( objectName.str().c_str() )->QueryIntAttribute("imagesNo", &imagesNo);
            dataset.categories[categoryNo].objects[objectNo].images.resize(imagesNo);
            dataset.categories[categoryNo].objects[objectNo].fullPaths.resize(imagesNo);
            dataset.categories[categoryNo].objects[objectNo].poses.resize(imagesNo);
            for (int imageNo=0;imageNo<imagesNo; imageNo++){
                std::stringstream imageName;
                imageName << "Image" << imageNo;
                dataset.categories[categoryNo].objects[objectNo].images[imageNo] = model->FirstChildElement( categoryName.str().c_str() )->FirstChildElement( objectName.str().c_str() )->FirstChildElement( imageName.str().c_str() )->Attribute( "name" );
                images.insert(std::make_pair(dataset.categories[categoryNo].objects[objectNo].images[imageNo],imageNo));
                std::string filePath(path + "/" + dataset.categories[categoryNo].name + "/" + dataset.categories[categoryNo].objects[objectNo].name + "/depth/" + dataset.categories[categoryNo].objects[objectNo].images[imageNo]);
                dataset.categories[categoryNo].objects[objectNo].fullPaths[imageNo] = filePath;
                dataset.categories[categoryNo].objects[objectNo].poses[imageNo] = readCameraPose(filePath);
                if (verbose>1)
                    std::cout << dataset.categories[categoryNo].objects[objectNo].images[imageNo] <<"\n";
            }
        }
    }
/*
    model->FirstChildElement( "parameters" )->QueryBoolAttribute("verbose", &verbose);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("overlapRf", &overlapRf);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("maxDepthValue", &maxDepthValue);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("scalingToMeters", &scalingToMeters);
    model->FirstChildElement( "parameters" )->QueryIntAttribute("backgroundOverlap", &backgroundOverlap);
    model->FirstChildElement( "parameters" )->QueryDoubleAttribute("boundaryResponseLevel", &boundaryResponseLevel);
*/
    if (verbose){
        std::cout << "Load Boris dataset parameters...\n";
        std::cout << "Categories no.: " << dataset.categories.size() << "\n";
        std::cout<< "Verbose: " << verbose << std::endl;
        std::cout<< "Dataset type: " << type << std::endl;
        std::cout<< "Dataset path: " << path << std::endl;
    }
 }

/// get image from dataset
void PacmanDataset::getDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const{
    if ((size_t)categoryNo<config.dataset.categories.size()){
        if ((size_t)objectNo<config.dataset.categories[categoryNo].objects.size()){
            if ((size_t)imageNo<config.dataset.categories[categoryNo].objects[objectNo].images.size()){
                readDepthImage(categoryNo, objectNo, imageNo, depthImage);
            }
            else{
                throw std::runtime_error("Wrong image no: " + std::to_string(categoryNo) + "->" + std::to_string(objectNo)  + "->" + std::to_string(imageNo) + "\n");
            }
        }
        else{
            throw std::runtime_error("Wrong object no: " + std::to_string(categoryNo) + "->" + std::to_string(objectNo) + "\n");
        }
    }
    else{
        throw std::runtime_error("Wrong category no: " + std::to_string(categoryNo) + "\n");
    }
}

/// get dataset info
void PacmanDataset::getDatasetInfo(hop3d::DatasetInfo& dataset) const{
    dataset = config.dataset;
}

/// read depth image from dataset
void PacmanDataset::readDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const{
    //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::string path = config.path + "/" + config.dataset.categories[categoryNo].name + "/" + config.dataset.categories[categoryNo].objects[objectNo].name + "/depth/" + config.dataset.categories[categoryNo].objects[objectNo].images[imageNo];
    if (config.verbose>1)
        std::cout << "load " << path << "\n";
    depthImage = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH );
    if(!depthImage.data ) {
        throw std::runtime_error("Could not open or find the image " + path + "\n");
    }
}

/// read number of point for the point cloud dataset
size_t PacmanDataset::getNumOfPoints(int categoryNo, int objectNo, int imageNo) const{
    cv::Mat depthImage;
    getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    size_t pointsNo=0;
    for (int i=0;i<depthImage.rows;i++){
        for (int j=0;j<depthImage.cols;j++){
            if (!std::isnan(double(depthImage.at<uint16_t>(i,j)))){
                pointsNo++;
            }
        }
    }
    return pointsNo;
}

/// get point from the point cloud
void PacmanDataset::getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const{
    cv::Mat depthImage;
    getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    bool found(false);
    size_t pointIter=0;
    double scale = 1/sensorModel.config.depthImageScale;
    for (int i=0;i<depthImage.rows;i++){
        for (int j=0;j<depthImage.cols;j++){
            if (!std::isnan(double(depthImage.at<uint16_t>(i,j)))){
                if (pointNo==pointIter){
                    sensorModel.getPoint(i, j, depthImage.at<uint16_t>(i,j)*scale, point);
                    found=true;
                    break;
                }
                pointIter++;
            }
        }
        if (found)
            break;
    }
    if (!found)
        point = Vec3(NAN, NAN, NAN);
}

/// read camera pose from file
Mat34 PacmanDataset::getCameraPose(int categoryNo, int objectNo, int imageNo) const{
    return config.dataset.categories[categoryNo].objects[objectNo].poses[imageNo];
}

/// translate path to filename into category, object, image numbers
void PacmanDataset::translateString(const std::string& path, int& categoryNo, int& objectNo, int& imageNo) const{
    if (isObjectInDataset(path, categoryNo, objectNo, imageNo))
        return;
    size_t found = path.find_last_of("/\\");
    std::string folder = path.substr(0,found);
    size_t foundFolder = folder.find_last_of("/\\");
    categoryNo = config.categories.at(folder.substr(foundFolder+1));
    objectNo = config.objects.at(folder.substr(foundFolder+1));
    imageNo = config.images.at(path.substr(found+1));
}

/// read camera pose from file
Mat34 PacmanDataset::readCameraPose(std::string& filename){
    size_t lastindex = filename.find_last_of(".");
    std::string rawname = filename.substr(0, lastindex);//remove '.png'
    std::replace( rawname.begin(), rawname.end(), '_', ' ');// replace '_' with space
    rawname.erase (std::remove(rawname.begin(), rawname.end(), 'P'), rawname.end());
    rawname.erase (std::remove(rawname.begin(), rawname.end(), 'I'), rawname.end());
    std::stringstream ss(rawname);
    int first, second, third;
    ss >> first >> second >> third;
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-idx2Angle[second]*3.14/180.0, Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(idx2Angle[third]*3.14/180.0, Eigen::Vector3d::UnitZ());
    Quaternion orientation(m);
    Mat34 cameraPose(Eigen::Translation<double, 3>(0,0,0.5)*Quaternion(orientation.w(),orientation.x(),orientation.y(),orientation.z()));
    return cameraPose;
}

/// add new data to the dataset
void PacmanDataset::addData(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, const std::string& categoryName, const std::string& imageName){
    std::cout << "Not implemented:\n" << categoryName << imageName << (*cloud).size();
}

/// check if object is in the dataset
bool PacmanDataset::isObjectInDataset(const std::string& imageName, int& categoryNo, int& objectNo, int& imageNo) const{
    categoryNo=0;
    for (auto cat : config.dataset.categories){
        objectNo=0;
        for (auto obj : cat.objects){
            imageNo=0;
            for (auto img : obj.images){
                if (imageName == img)
                    return true;
                imageNo++;
            }
            objectNo++;
        }
        categoryNo++;
    }
    return false;
}

std::unique_ptr<hop3d::Dataset> hop3d::createPacmanDataset(void) {
    //datasetPacman.reset(new PacmanDataset());
    //return datasetPacman.get();
	return hop3d::make_unique<PacmanDataset>();
}

std::unique_ptr<hop3d::Dataset> hop3d::createPacmanDataset(std::string config, std::string sensorConfig) {
    //datasetPacman.reset(new PacmanDataset(config, sensorConfig));
    //return datasetPacman.get();
	return hop3d::make_unique<PacmanDataset>(config, sensorConfig);
}
