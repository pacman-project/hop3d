#include <iostream>
#include <vector>

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "ImageFilter/imageFilter.h"
#include "ImageFilter/normalImageFilter.h"
#include "Dataset/datasetBoris.h"

using namespace std;


int main(void)
{
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string configFile(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
        std::string sensorConfigFile(config.FirstChildElement( "CameraModel" )->Attribute( "configFilename" ));
        std::string datasetConfigFile(config.FirstChildElement( "Dataset" )->Attribute( "configFilename" ));

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

        std::vector<cv::Mat> vecImages;
        hop3d::Reader reader;
        reader.readMultipleImages("../../resources/depthImages",vecImages);

        hop3d::Dataset* dataset = hop3d::createBorisDataset(datasetConfigFile,sensorConfigFile);
        dataset->getDepthImage(0,0,0,vecImages[0]);

        std::vector<hop3d::Octet> octets;
        filter->computeOctets(vecImages[0],0,0,0,octets);

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
