#include "hop3d/Data/Graph.h"
#include "tinyXML/tinyxml2.h"
#include <set>
#include <fstream>

namespace hop3d {

/// Construction
Hierarchy::Hierarchy(std::string configFilename) {
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load hierarchy config file: " + filename);
    tinyxml2::XMLElement *params = config.FirstChildElement("Hierarchy")->FirstChildElement("parameters");
    int VDLayersNo, VIndLayersNo;
    params->QueryIntAttribute("viewDependentLayersNo", &VDLayersNo);
    params->QueryIntAttribute("viewIndependentLayersNo", &VIndLayersNo);
    params->QueryIntAttribute("viewDepPartsFromLayerNo", &viewDepPartsFromLayerNo);

    this->viewDependentLayers.resize(VDLayersNo);
    this->viewIndependentLayers.resize(VIndLayersNo);
}

// Insertion operator
std::ostream& operator<<(std::ostream& os, const Hierarchy& hierarchy){
    os << hierarchy.viewDepPartsFromLayerNo << "\n";
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
    // interpreter size
    os << hierarchy.interpreter.size() << "\n";
    for (auto& element : hierarchy.interpreter){
        os << element.first << " " << element.second << " ";
    }
    os << "\n";
    os << hierarchy.viewIndependentLayers.size() << "\n";
    for (size_t i = 0;i<hierarchy.viewIndependentLayers.size();i++){
        os << hierarchy.viewIndependentLayers[i].size() << "\n";
        for (auto& part : hierarchy.viewIndependentLayers[i]){
            os << part;
        }
    }
    os << "\n";
    os << hierarchy.graph.size() << "\n";
    for (auto &node : hierarchy.graph){
            os << node.first << "\n";
            os << node.second.size() << "\n";
            for (auto &incId : node.second){
                os << incId << " ";
            }
            std::cout << "\n";
    }
    os << "\n";
    return os;
}

// Extraction operator
std::istream& operator>>(std::istream& is, Hierarchy& hierarchy){
    // read filters no
    is >> hierarchy.viewDepPartsFromLayerNo;
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
    // interpreter size
    int interpreterSize;
    is >> interpreterSize;
    hierarchy.interpreter.clear();
    for (int i=0;i<interpreterSize;i++){
        int firstId, secondId;
        is >> firstId >> secondId;
        hierarchy.interpreter.insert(std::make_pair(firstId, secondId));
    }
    int viewIndepLayersNo;
    is >> viewIndepLayersNo;
    hierarchy.viewIndependentLayers.clear();
    hierarchy.viewIndependentLayers.resize(viewIndepLayersNo);
    for (int i = 0;i<viewIndepLayersNo;i++){
        int partsNo;
        is >> partsNo;
        for (int j = 0;j<partsNo;j++){
            ViewIndependentPart part;
            is >> part;
            hierarchy.viewIndependentLayers[i].push_back(part);
        }
    }
    hierarchy.graph.clear();
    int graphSize;
    is >> graphSize;
    for (int i=0;i<graphSize; i++){
        std::uint32_t partId;
        is >> partId;
        int elementsNo;
        is >> elementsNo;
        std::vector<std::uint32_t> elementsIds(elementsNo,0);
        for (int j=0;j<elementsNo;j++){
            std::uint32_t compId;
            is >> compId;
            elementsIds[j] = compId;
        }
        hierarchy.graph.insert(std::make_pair(partId,elementsIds));
    }
    return is;
}

/// show info
void Hierarchy::showInfo(void){
    std::cout << "First layer size: " << this->firstLayer.size() << "\n";
    for (size_t i=0;i<viewDependentLayers.size();i++){
        std::cout << "VD layer " << i+1 << " size: " << viewDependentLayers[i].size() << "\n";
    }
    for (size_t i=0;i<viewIndependentLayers.size();i++){
        std::cout << "Volumetric layer " << i+1 << " size: " << viewIndependentLayers[i].size() << "\n";
    }
}

/// add parts to existing clusters
void Hierarchy::addParts(const ViewDependentPart::Seq& parts, int layerNo){
    for (const auto& part : parts)
        viewDependentLayers[layerNo][part.id].group.push_back(part);
}

/// add parts to existing clusters
void Hierarchy::addParts(const ViewIndependentPart::Seq& parts, int layerNo){
    for (const auto& part : parts){
        for (auto& partVoc : viewIndependentLayers[layerNo]){
            if (part.id==partVoc.id){
                partVoc.group.push_back(part);
                break;
            }
        }
    }
}

/// add new parts (new clusters)
void Hierarchy::addNewParts(const ViewDependentPart::Seq& parts, int layerNo){
    for (const auto& part : parts){
        ViewDependentPart pAdd(part);
        pAdd.id = viewDependentLayers[layerNo].back().id+1;
        viewDependentLayers[layerNo].push_back(pAdd);
    }
}

/// add new parts (new clusters)
void Hierarchy::addNewParts(const ViewIndependentPart::Seq& parts, int layerNo){
    for (const auto& part : parts){
        ViewIndependentPart pAdd(part);
        pAdd.id = viewIndependentLayers[layerNo].back().id+1;
        viewIndependentLayers[layerNo].push_back(pAdd);
    }
}

/// get normal vector related to the part
void Hierarchy::getNormal(const ViewDependentPart& part, Vec3& normal) const{
    if (part.layerId>2){
        if (part.partIds[1][1]!=-1){
            //getNormal(viewDependentLayers[part.layerId-3][part.partIds[1][1]], normal);
            computeMeanVector(part, normal);
        }
        else{
            //normal=Vec3(0,0,-1);
            computeMeanVector(part, normal);
        }
    }
    else {
        if (part.partIds[1][1]!=-1) {
            computeMeanVector(part, normal);
            //normal = firstLayer[part.partIds[1][1]].normal;
        }
        else{
            computeMeanVector(part, normal);
            //normal=Vec3(0,0,-1);
            //std::cout << "normal " << normal.transpose() << " ";
        }
    }
}

/// compute mean vector for 2nd layer part
void Hierarchy::computeMeanVector(const ViewDependentPart& part, Vec3& normal) const{
    Vec3 mean(0,0,0);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            if (part.partIds[i][j]!=-1){
                if (part.layerId>2){
                    Vec3 normTmp;
                    getNormal(viewDependentLayers[part.layerId-3][part.partIds[i][j]], normTmp);
                    mean += normTmp;
                }
                else
                    mean += part.partsPosNorm[i][j].mean.block<3,1>(3,0);
                    //mean += firstLayer[part.partIds[i][j]].normal;
            }
        }
    }
    normal = mean.normalized();
}

/// get points related to the part assuming that we have flat patches
/*void Hierarchy::getPoints(const ViewDependentPart& part, std::vector<Vec3>& points) const{
  todo  if (part.layerId>2)
        getPoints(viewDependentLayers[part.layerId-3][part.partIds[1][1]], points);
    else
        normal = firstLayer[part.partIds[1][1]].normal;

}*/

/// compute graph from hierarchy structure
void Hierarchy::computeGraph(IndexSeqMap& hierarchyGraph){
    int layerInc = 10000;
    graph.clear();
    graph.insert(std::make_pair(0,std::vector<unsigned int>()));
    int layerNo = 0;
    for (const auto &layer : viewDependentLayers){//graph for view-dependent layers
        int wordId = 0;
        for (const auto &word : layer){
            if (layerNo==0){
                graph.insert(std::make_pair(((layerNo+1)*layerInc)+wordId,std::vector<unsigned int>(1,0)));
            }
            else {
                std::set<std::uint32_t> incomingIds;
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        if (word.partIds[i][j]>-1)
                            incomingIds.insert(incomingIds.end(),((layerNo)*layerInc)+word.partIds[i][j]);
                    }
                }
                std::vector<std::uint32_t> inputIds(incomingIds.begin(), incomingIds.end());
                graph.insert(std::make_pair(((layerNo+1)*layerInc)+wordId,inputIds));
            }
            wordId++;
        }
        layerNo++;
    }
    layerNo = 0;
    for (const auto &layer : viewIndependentLayers){//graph for view-dependent layers
        std::cout << "layer no " << layerNo << " size " << layer.size() << "\n";
        int wordId = 0;
        for (const auto &word : layer){
            std::set<std::uint32_t> incomingIds;
            for (auto &id : word.incomingIds){
                if (layerNo == 0)
                    incomingIds.insert(incomingIds.end(),((((int)viewDepPartsFromLayerNo)*layerInc)+id));
                else
                    incomingIds.insert(incomingIds.end(),((((int)viewDependentLayers.size()+layerNo)*layerInc)+id));
            }
            std::vector<std::uint32_t> inputIds(incomingIds.begin(), incomingIds.end());
            graph.insert(std::make_pair((((int)viewDependentLayers.size()+layerNo+1)*layerInc)+wordId,inputIds));
            wordId++;
        }
        layerNo++;
    }
    // print hierarchy
    /*for (auto &node : graph){
        std::cout << "part id " << node.first << "\n";
        std::cout << "is built from parts: ";
        for (auto &incId : node.second){
            std::cout << incId << ", ";
        }
        std::cout << "\n";
    }*/
    hierarchyGraph = graph;
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
