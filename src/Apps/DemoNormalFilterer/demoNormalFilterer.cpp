#include <iostream>
#include <vector>

#include "hop3d/Data/Defs.h"
#include "hop3d/Data/Cloud.h"
#include "hop3d/ImageFilter/imageFilter.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include "hop3d/Dataset/datasetBoris.h"
#include "hop3d/Dataset/datasetPacman.h"

using namespace std;


int main(void)
{
    try {
        tinyxml2::XMLDocument config;
        std::string prefix("../../resources/");
        config.LoadFile(std::string(prefix +"hop3dConfigGlobal.xml").c_str());
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string configFile(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
        std::string sensorConfigFile(config.FirstChildElement( "CameraModel" )->Attribute( "configFilename" ));
        std::string datasetConfigFile(config.FirstChildElement( "Dataset" )->Attribute( "configFilename" ));
        configFile = prefix+configFile;
        sensorConfigFile = prefix+sensorConfigFile;
        datasetConfigFile = prefix+datasetConfigFile;

        int datasetType;
        config.FirstChildElement( "Dataset" )->QueryIntAttribute("datasetType", &datasetType);

        int filterType;
        config.FirstChildElement( "Filterer" )->QueryIntAttribute("filterType", &filterType);

        hop3d::ImageFilter* filter;
        if (filterType==hop3d::ImageFilter::FILTER_DEPTH){
            std::cout << "Select normal filter in config file.\n";
            return 0;
        }
        else if (filterType==hop3d::ImageFilter::FILTER_NORMAL){

        }
        filter = hop3d::createNormalImageFilter(configFile, sensorConfigFile);

        std::vector<cv::Mat> vecImages(1);

        std::unique_ptr<hop3d::Dataset> dataset;
        if (datasetType==hop3d::Dataset::DATASET_BORIS){
            dataset = hop3d::createBorisDataset(datasetConfigFile,sensorConfigFile);
        }
        else if (datasetType==hop3d::Dataset::DATASET_PACMAN){
            dataset = hop3d::createPacmanDataset(datasetConfigFile,sensorConfigFile);
        }
        else {// default dataset
            dataset = hop3d::createBorisDataset(datasetConfigFile,sensorConfigFile);
        }

        dataset->getDepthImage(0,0,0,vecImages[0]);

        std::vector<hop3d::Octet> octets;
        filter->computeOctets(vecImages[0],0,0,0,octets, false);

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
