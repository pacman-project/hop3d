#include <iostream>
#include <vector>

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "Utilities/Reader.h"
#include "Utilities/Writer.h"
#include "Visualizer/Visualizer.h"

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

Visualizer visualizer(1024,768,"testing vis");

visualizer.loadVertices("uvmap.DDS","suzanne.obj");

do{

    visualizer.render();
} // Check if the ESC key was pressed or the window was closed
while( visualizer.checkClose());

visualizer.close();



return 0;
}
