#include "../include/Dataset/datasetBoris.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace hop3d;

/// A single instance of BorisDataset
BorisDataset::Ptr dataset;

BorisDataset::BorisDataset(void) : Dataset("Boris dataset", DATASET_BORIS) {
}

/// Construction
BorisDataset::BorisDataset(std::string config, std::string configSensor) :
        Dataset("Boris dataset", DATASET_BORIS), config(config), sensorModel(configSensor) {
}

/// Destruction
BorisDataset::~BorisDataset(void) {
}

///config class constructor
BorisDataset::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Boris dataset config file: " << filename << ".\n";
    tinyxml2::XMLElement * model = config.FirstChildElement( "Dataset" );
    model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    int categoriesNo;
    model->FirstChildElement( "parameters" )->QueryIntAttribute("categoriesNo", &categoriesNo);
    dataset.categories.resize(categoriesNo);
    type = model->FirstChildElement( "parameters" )->Attribute( "type" );
    path = model->FirstChildElement( "parameters" )->Attribute( "path" );
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
            if (verbose>1)
                std::cout << dataset.categories[categoryNo].objects[objectNo].name <<"\n";
            int imagesNo;
            model->FirstChildElement( categoryName.str().c_str() )->FirstChildElement( objectName.str().c_str() )->QueryIntAttribute("imagesNo", &imagesNo);
            dataset.categories[categoryNo].objects[objectNo].images.resize(imagesNo);
            dataset.categories[categoryNo].objects[objectNo].poses.resize(imagesNo);
            for (int imageNo=0;imageNo<imagesNo; imageNo++){
                std::stringstream imageName;
                imageName << "Image" << imageNo;
                dataset.categories[categoryNo].objects[objectNo].images[imageNo] = model->FirstChildElement( categoryName.str().c_str() )->FirstChildElement( objectName.str().c_str() )->FirstChildElement( imageName.str().c_str() )->Attribute( "name" );
                std::string filePath(path + "/" + dataset.categories[categoryNo].objects[objectNo].name + "/" + dataset.categories[categoryNo].objects[objectNo].images[imageNo]);
                dataset.categories[categoryNo].objects[objectNo].poses[imageNo] = getCameraPose(filePath);
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
void BorisDataset::getDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const{
    if ((size_t)categoryNo<config.dataset.categories.size()){
        if ((size_t)objectNo<config.dataset.categories[categoryNo].objects.size()){
            if ((size_t)imageNo<config.dataset.categories[categoryNo].objects[objectNo].images.size()){
                readDepthImage(categoryNo, objectNo, imageNo, depthImage);
            }
            else{
                std::cout << "Wrong image no: " << categoryNo << "->" << objectNo  << "->" << imageNo << "\n";
                getchar();
            }
        }
        else{
            std::cout << "Wrong object no: " << categoryNo << "->" << objectNo << "\n";
            getchar();
        }
    }
    else{
        std::cout << "Wrong category no: " << categoryNo << "\n";
        getchar();
    }
}

/// get dataset info
void BorisDataset::getDatasetInfo(hop3d::DatasetInfo& dataset) const{
    dataset = config.dataset;
}

/// read depth image from dataset
void BorisDataset::readDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::string path = config.path + "/" + config.dataset.categories[categoryNo].objects[objectNo].name + "/" + config.dataset.categories[categoryNo].objects[objectNo].images[imageNo];
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (path, *cloud) == -1){//* load the file
        PCL_ERROR ("Couldn't read pcd file \n");
    }
    Mat34 cameraPose(Eigen::Translation<double, 3>(cloud->sensor_origin_(0), cloud->sensor_origin_(1), cloud->sensor_origin_(2))*Quaternion(cloud->sensor_orientation_));
    Mat34 cameraPoseInv(cameraPose.inverse());
    cv::Mat image(cloud->height,cloud->width, CV_16U,cv::Scalar(0));
    for (size_t i = 0; i < cloud->points.size(); ++i){
        if (!std::isnan(cloud->points[i].x)){
            Mat34 point(Eigen::Translation<double, 3>(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z)*Mat33::Identity());
            Mat34 pointCam=cameraPoseInv*point;
            //Eigen::Vector3d coordImg = sensorModel.inverseModel(pointCam(0,3),pointCam(1,3),pointCam(2,3));
            /*std::cout << "point3d " << pointCam(0,3) << ", " << pointCam(1,3) << ", " << pointCam(2,3) << "\n";
            std::cout << "coordImg " << coordImg.transpose() << "\n";
            Eigen::Vector3d coord3d; sensorModel.getPoint(coordImg,coord3d);
            std::cout << "coord3d " << coord3d.transpose() << "\n";
            getchar();*/
            //image.at<uint16_t>((unsigned int)coordImg(0), (unsigned int)coordImg(1)) = (short unsigned int)((double)coordImg(2)*sensorModel.config.depthImageScale);
            image.at<uint16_t>((unsigned int)i/cloud->width, (unsigned int)i%cloud->width) = (short unsigned int)(pointCam(2,3)*sensorModel.config.depthImageScale);
        }
    }
    depthImage = image;
}

/// read camera pose from file
Mat34 BorisDataset::getCameraPose(std::string& filename){
    pcl::PCLPointCloud2 cloud;    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;    int pcd_version;
    int data_type;    unsigned int data_idx;
    pcl::PCDReader reader;
    if (reader.readHeader(filename, cloud, origin, orientation, pcd_version, data_type, data_idx)){
        PCL_ERROR ("Couldn't read pcd file \n");
    }
    return Mat34(Eigen::Translation<double, 3>(origin(0),origin(1),origin(2))*Quaternion(orientation.w(),orientation.x(),orientation.y(),orientation.z()));
}

hop3d::Dataset* hop3d::createBorisDataset(void) {
    dataset.reset(new BorisDataset());
    return dataset.get();
}

hop3d::Dataset* hop3d::createBorisDataset(std::string config, std::string sensorConfig) {
    dataset.reset(new BorisDataset(config, sensorConfig));
    return dataset.get();
}
