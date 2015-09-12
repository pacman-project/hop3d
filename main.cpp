#include <iostream>
#include "Data/Defs.h"
#include "Utilities/Reader.h"
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

        hop3d::ImageFilter *imageFilter;
        imageFilter = hop3d::createDepthImageFilter(filterConfig);
        std::cout << imageFilter->getName() << "\n";

        hop3d::Hierarchy hierarchy("../../resources/configGlobal.xml");
        std::cout << "Hierarchy: number of view-dependent layers: " << hierarchy.viewDependentLayers.size() << "\n";
        std::cout << "Finished" << std::endl;
        hop3d::Filter::Seq filters;
        hop3d::Reader reader;
        reader.readFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml",filters);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

