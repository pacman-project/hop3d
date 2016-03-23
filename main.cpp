#include <iostream>
#include "hop3d/HOP3D/HOP3DBham.h"

int main(void){
    try {
        tinyxml2::XMLDocument config;
        std::string prefix("../../resources/");
        config.LoadFile(std::string(prefix+"hop3dConfigGlobal.xml").c_str());
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        bool train, load, inference, loadInferenceResults;
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("train", &train);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("load", &load);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("inference", &inference);
        config.FirstChildElement("Hierarchy")->FirstChildElement("parameters")->QueryBoolAttribute("loadInferenceResults", &loadInferenceResults);
        std::string file2load(config.FirstChildElement( "Hierarchy" )->FirstChildElement("parameters")->Attribute( "file2load" ));
        std::string inferenceFile(config.FirstChildElement( "Hierarchy" )->FirstChildElement("parameters")->Attribute( "inferenceFile" ));

        hop3d::HOP3D* lhop3d = hop3d::createHOP3DBham(prefix + "hop3dConfigGlobal.xml");
        if (train)
            lhop3d->learn();
        if (load)
            lhop3d->load(file2load);
        if (inference){
            lhop3d->inference();
            /*std::vector<std::pair<cv::Mat, hop3d::Mat34>> frames;
            std::string path = "depth.png";
            cv::Mat depthImage = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH );
            std::cout << depthImage.rows << ", " << depthImage.cols << "vvv\n";
            if(!depthImage.data ) {
                throw std::runtime_error("Could not open or find the image " + path + "\n");
            }
            frames.push_back(std::make_pair(depthImage, hop3d::Mat34::Identity()));
            lhop3d->inference(frames,0,0);*/
        }
        if (loadInferenceResults)
            lhop3d->loadInference(inferenceFile);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

