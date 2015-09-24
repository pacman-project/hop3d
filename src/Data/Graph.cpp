#include "Data/Graph.h"
#include "../../external/tinyXML/tinyxml2.h"

namespace hop3d {

/// Construction
Hierarchy::Hierarchy(std::string configFilename) {
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID()){
        std::cout << "unable to load hierarchy config file.\n";
    }
    tinyxml2::XMLElement *params = config.FirstChildElement("Hierarchy")->FirstChildElement("parameters");
    int VDLayersNo, VIndLayersNo;
    params->QueryIntAttribute("viewDependentLayersNo", &VDLayersNo);
    params->QueryIntAttribute("viewIndependentLayersNo", &VIndLayersNo);

    this->viewDependentLayers.resize(VDLayersNo);
    this->viewIndependentLayers.resize(VIndLayersNo);
}

/// get normal vector related to the part
void Hierarchy::getNormal(const ViewDependentPart& part, Vec3& normal) const{
    int layerId = part.layerId;
    if (layerId==2) {
        int filterId = part.partIds[1][1];
        normal = firstLayer[filterId].normal;
    }
    else if (layerId==3) {
        int filterId = viewDependentLayers[0][part.partIds[1][1]].partIds[1][1];
        normal = firstLayer[filterId].normal;
    }
}

}
