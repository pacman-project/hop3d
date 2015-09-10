#include <iostream>

#include "Data/Defs.h"
#include "Utilities/Reader.h"
#include "ImageFilter/depthImageFilter.h"


int main(void){
    hop3d::ImageFilter *imageFilter;
    imageFilter = hop3d::createDepthImageFilter("depthFilter.xml");
    std::cout << imageFilter->getName() << "\n";
    std::cout << "Hello World" << std::endl;
    std::vector<hop3d::Filter> filters;
    hop3d::Reader reader;
    reader.readFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml",filters);
    return 0;
}

