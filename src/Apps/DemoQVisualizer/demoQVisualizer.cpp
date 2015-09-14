#include "../include/Data/Defs.h"
#include "../external/tinyXML/tinyxml2.h"
#include "../include/QVisualizer/Qvisualizer.h"
#include <GL/glut.h>
#include <qapplication.h>
#include <iostream>
#include <thread>

using namespace std;

// run HOP3D
void runHOP3D(){
    std::cout << "Press Enter to start\n";
    getchar();
    //slam.get()->startProcessing();
}

int main(int argc, char** argv)
{
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string configFile(config.FirstChildElement( "Visualizer" )->Attribute( "configFilename" ));
        QGLVisualizer::Config configVis(configFile);//something is wrong with QApplication when Qapplication
        //object is created. libTinyxml can read only ints from xml file

        QApplication application(argc,argv);
        glutInit(&argc, argv);

        QGLVisualizer visu(configVis);

        visu.setWindowTitle("HOP3D hierarchy viewer");

        // Make the viewer window visible on screen.
        visu.show();
        //slam.get()->attachVisualizer(&visu);

        // run HOP3D
        std::thread tHOP3D(runHOP3D);

        // Run main loop.
        application.exec();
        tHOP3D.join();

        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }

    return 0;
}