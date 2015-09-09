#include <iostream>
#include "include/ImageFilter/depthImageFilter.h"

int main(void){
    ImageFilter *imageFilter;
    imageFilter = createDepthImageFilter("depthFilter.xml");
    std::cout << imageFilter->getName() << "\n";
    std::cout << "Hello World" << std::endl;
    return 0;
}

