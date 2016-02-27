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
        if (inference)
            lhop3d->inference();
        if (loadInferenceResults)
            lhop3d->loadInference(inferenceFile);
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

