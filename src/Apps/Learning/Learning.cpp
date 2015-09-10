#include <iostream>
#include <vector>

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "Data/Vocabulary.h"
#include "Utilities/Reader.h"
#include "Utilities/Writer.h"
#include "Core/LayerFilterer.h"


int main( void )
{

std::cout << "Hello World" << std::endl;
std::clog << "Hello World" << std::endl;

hop3d::Reader reader;
hop3d::Writer writer;
hop3d::PointCloud pointCloud;
std::vector<Eigen::Vector4i> faces;
hop3d::LayerVocabulary first;

reader.readPlyFile(std::string("cube.ply"), pointCloud, faces );
writer.writePlyToFile(std::string("cubeOut.ply"), pointCloud, faces);
//reader.readFirstLayerVoc("crude_400.txt", first.FirstLayerPartVec);

/*for(auto iter : first.FirstLayerPartVec){
    std::cout << iter.getPartId() << std::endl;
    std::cout << iter.getNormal() << std::endl;
}*/


LayerFilterer layerFilterer;
//layerFilterer.nearestNeighbour(pointCloud);
layerFilterer.radiusSearch(pointCloud,0.01f);

return 0;
}


