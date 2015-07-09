#include <iostream>
#include <vector>

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "Utilities/Reader.h"
#include "Utilities/Writer.h"
#include "Core/LayerFilterer.h"


int main( int argc, char** argv )
{

std::cout << "Hello World" << std::endl;
std::clog << "Hello World" << std::endl;

Reader reader;
Writer writer;
PointCloud pointCloud;
std::vector<hop3d::Face> faces;

reader.readPlyFile(std::string("cube.ply"), pointCloud, faces );
writer.writePlyToFile(std::string("cubeOut.ply"), pointCloud, faces);

LayerFilterer layerFilterer;
//layerFilterer.nearestNeighbour(pointCloud);
layerFilterer.radiusSearch(pointCloud,0.01);

return 0;
}


