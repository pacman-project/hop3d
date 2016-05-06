#include "hop3d/Data/Defs.h"
#include "tinyXML/tinyxml2.h"
#include "hop3d/QVisualizer/Qvisualizer.h"
#include "hop3d/HOP3D/HOP3DBham.h"
#include <GL/glut.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <qapplication.h>
#include <iostream>
#include <thread>

using namespace std;

hop3d::HOP3D* lhop3d;
// run HOP3D
void runHOP3D(bool train, bool load, bool inference, bool loadInference, std::string file2load, std::string inferenceFile){
    if (train)
        lhop3d->learn();
    if (load)
        lhop3d->load(file2load);
    if (inference){
        lhop3d->inference();
        /*std::vector<std::pair<cv::Mat, hop3d::Mat34>> frames;
        std::string path = "depth.png";
        cv::Mat depthImage = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH );
        if(!depthImage.data ) {
            throw std::runtime_error("Could not open or find the image " + path + "\n");
        }
        frames.push_back(std::make_pair(depthImage, hop3d::Mat34::Identity()));
        lhop3d->inference(frames,0,0);*/
//        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//        std::string path = "cloudInf.pcd";
//        if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (path, *cloud) == -1){//* load the file
//            PCL_ERROR ("Couldn't read pcd file \n");
//        }
//        std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds;
//        clouds.insert(std::make_pair("mrmuscle.pcd",cloud));
//        lhop3d->inference(clouds);
    }
    if (loadInference)
        lhop3d->loadInference(inferenceFile);
}

int main(int argc, char** argv)
{
    try {
        tinyxml2::XMLDocument config;
        std::string prefix("../../resources/");
        config.LoadFile(std::string(prefix+"hop3dConfigGlobal.xml").c_str());
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string configFile(prefix+config.FirstChildElement( "QVisualizer" )->Attribute( "configFilename" ));
        bool train, load, inference, loadInferenceResults;
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("train", &train);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("load", &load);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("inference", &inference);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("loadInferenceResults", &loadInferenceResults);
        std::string file2load(config.FirstChildElement( "Hierarchy" )->FirstChildElement("parameters")->Attribute( "file2load" ));
        std::string inferenceFile(config.FirstChildElement( "Hierarchy" )->FirstChildElement("parameters")->Attribute( "inferenceFile" ));

        lhop3d = hop3d::createHOP3DBham(prefix + "hop3dConfigGlobal.xml");
        QGLVisualizer::Config configVis(configFile);//something is wrong with QApplication when Qapplication
        //object is created. libTinyxml can read only ints from xml file
        // found: Qt changes locale settings use

        QApplication application(argc,argv);
        setlocale(LC_NUMERIC,"C");
        glutInit(&argc, argv);
        QGLVisualizer visu(configVis);

        visu.setWindowTitle("HOP3D hierarchy viewer");
        // Make the viewer window visible on screen.
        visu.show();
        ((hop3d::HOP3DBham*)lhop3d)->attachVisualizer(&visu);
        // run HOP3D
        std::thread tHOP3D(runHOP3D, train, load, inference, loadInferenceResults, file2load, inferenceFile);
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
