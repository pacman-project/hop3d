#include <iostream>
#include <vector>

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "Utilities/Reader.h"
#include "Utilities/Writer.h"
#include "Visualizer/Visualizer.h"

int main( void)
{



std::cout << "Hello World" << std::endl;
std::clog << "Hello World" << std::endl;

hop3d::Reader reader;
hop3d::Writer writer;
hop3d::PointCloud pointCloud;
std::vector<hop3d::Face> faces;

reader.readPlyFile(std::string("cube.ply"), pointCloud, faces );
writer.writePlyToFile(std::string("cylinderOut.ply"), pointCloud, faces);

Visualizer visualizer(900,600,"testing vis");

visualizer.loadPoints("uvmap.DDS",pointCloud);
//visualizer.loadVertices("uvmap.DDS","suzanne.obj");
//visualizer.createSphere("uvmap.DDS",0.05,10);
visualizer.createEllipse("uvmap.DDS",0.05,0.05,20);
do{

    visualizer.renderPoints(pointCloud);
    //visualizer.render();
} // Check if the ESC key was pressed or the window was closed
while( visualizer.checkClose());

visualizer.close();



return 0;
}
