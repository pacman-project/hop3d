#include <iostream>
#include "ImageFilter/depthImageFilter.h"
#include "Data/Graph.h"

int main(void){
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string filterConfig(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));

        ImageFilter *imageFilter;
        imageFilter = createDepthImageFilter(filterConfig);
        std::cout << imageFilter->getName() << "\n";

        Hierarchy hierarchy("../../resources/configGlobal.xml");
        std::cout << "Hierarchy: number of view-dependent layers: " << hierarchy.viewDependentLayers.size() << "\n";
        std::cout << "Finished" << std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

