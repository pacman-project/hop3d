#include <iostream>
#include "include/ImageFilter/depthImageFilter.h"

int main(void){
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string filtererType(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));

        ImageFilter *imageFilter;
        imageFilter = createDepthImageFilter(filtererType);
        std::cout << imageFilter->getName() << "\n";
        std::cout << "Hello World" << std::endl;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

