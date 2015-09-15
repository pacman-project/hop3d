#include "Data/Defs.h"
#include "../external/tinyXML/tinyxml2.h"
#include "QVisualizer/Qvisualizer.h"
#include "HOP3D/HOP3DBham.h"
#include <GL/glut.h>
#include <qapplication.h>
#include <iostream>
#include <thread>

using namespace std;

hop3d::HOP3D* lhop3d;
// run HOP3D
void runHOP3D(){
    std::cout << "Press Enter to start\n";
    getchar();
    lhop3d->learn();
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
        std::string configFile(config.FirstChildElement( "QVisualizer" )->Attribute( "configFilename" ));

        lhop3d = hop3d::createHOP3DBham("configGlobal.xml");

        QGLVisualizer::Config configVis(configFile);//something is wrong with QApplication when Qapplication
        //object is created. libTinyxml can read only ints from xml file

        QApplication application(argc,argv);
        glutInit(&argc, argv);

        QGLVisualizer visu(configVis);

        visu.setWindowTitle("HOP3D hierarchy viewer");

        // Make the viewer window visible on screen.
        visu.show();

        ((hop3d::HOP3DBham*)lhop3d)->attachVisualizer(&visu);

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
