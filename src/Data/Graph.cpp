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

// Insertion operator
std::ostream& operator<<(std::ostream& os, const Hierarchy& hierarchy){
    //save filters no
    os << hierarchy.firstLayer.size() << "\n";
    for (auto& filter : hierarchy.firstLayer){
        os << filter;
    }
    os << hierarchy.viewDependentLayers.size() << "\n";
    for (size_t i = 0;i<hierarchy.viewDependentLayers.size();i++){
        os << hierarchy.viewDependentLayers[i].size() << "\n";
        for (auto& part : hierarchy.viewDependentLayers[i]){
            os << part;
        }
    }
    return os;
}

// Extraction operator
std::istream& operator>>(std::istream& is, Hierarchy& hierarchy){
    // read filters no
    int filtersNo;
    is >> filtersNo;
    hierarchy.firstLayer.clear();
    hierarchy.firstLayer.reserve(filtersNo);
    for (int i=0;i<filtersNo;i++){
        Filter filterTmp;
        is >> filterTmp;
        hierarchy.firstLayer.push_back(filterTmp);
    }
    int viewDepLayersNo;
    is >> viewDepLayersNo;
    hierarchy.viewDependentLayers.clear();
    hierarchy.viewDependentLayers.resize(viewDepLayersNo);
    for (int i = 0;i<viewDepLayersNo;i++){
        int partsNo;
        is >> partsNo;
        for (int j = 0;j<partsNo;j++){
            ViewDependentPart part;
            is >> part;
            hierarchy.viewDependentLayers[i].push_back(part);
        }
    }
    return is;
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

/// print ids
void Hierarchy::printIds(const ViewDependentPart& part){
    if (part.layerId==2){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                std::cout << part.partIds[0][0] << ", ";
            }
            std::cout << "\n";
        }
    }
    else if (part.layerId==3){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                for (int k=0;k<3;k++){
                    for (int l=0;l<3;l++){
                        if (part.partIds[i][k]==-1)
                            std::cout << "-1, ";
                        else
                            std::cout << viewDependentLayers[0][part.partIds[i][k]].partIds[j][l] << ", ";
                    }
                }
                std::cout << "\n";
                //(0,0,0,0) (0,0,0,1) (0,0,0,2) (0,1,0,0) (0,1,0,1) (0,1,0,2) (0,2,0,0) (0,2,0,1) (0,2,0,2)
                //(0,0,1,0) (0,0,1,1) (0,0,1,2) (0,1,1,0) (0,1,1,1) (0,1,1,2) (0,2,1,0) (0,2,1,1) (0,2,1,2)
                //(0,0,2,0) (0,0,2,1) (0,0,2,2) (0,1,2,0) (0,1,2,1) (0,1,2,2) (0,2,2,0) (0,2,2,1) (0,2,2,2)
            }
        }
    }
}

}
