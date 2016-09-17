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

/// compute statisticts for selected layer (results are written to representative parts)
void Hierarchy::computeVDStats(int layerNo){
    if (layerNo>=(int)viewDependentLayers.size()){
        throw std::runtime_error("Compute stats: wrong layer no \n");
    }
    else{
        for (auto& part : viewDependentLayers[layerNo]){
            std::array<std::array<ViewDependentPart::SubpartsProb,3>,3> subpartsProb;
            std::array<std::array<GaussianSE3,3>,3> subpartsPos;
            std::array<std::array<int,3>,3> subpartsNo = {{{0,0,0},{0,0,0},{0,0,0}}};
            std::vector<std::pair<int, int>> pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
            std::vector<std::pair<int,Mat34>> optRot;
            for (auto& partFromCluster : part.group){
                Mat34 estTrans;
                int rotIdx;
                if (layerNo==0)
                    ViewDependentPart::distanceInvariant(part,partFromCluster,3,estTrans, rotIdx);
                else if (layerNo==1)
                    ViewDependentPart::distanceInvariant(part,partFromCluster,3, viewDependentLayers[0],estTrans, rotIdx);
                else if (layerNo==2)
                    ViewDependentPart::distanceInvariant(part,partFromCluster,3, viewDependentLayers[0], viewDependentLayers[1],estTrans, rotIdx);
                estTrans=estTrans.inverse();
                optRot.push_back(std::make_pair(rotIdx,estTrans));
                size_t idx=rotIdx;
                for (size_t j=0;j<pointCorrespondence.size();j++){
                    int coordA[2]={pointCorrespondence[j].first, pointCorrespondence[j].second};//partA is not rotated
                    int coordB[2]={pointCorrespondence[idx%(pointCorrespondence.size())].first, pointCorrespondence[idx%(pointCorrespondence.size())].second};//partA is not rotated
                    //std::cout << "correspondence " << coordA[0] << ", " << coordA[1] << " -> " << coordB[0] << ", " << coordB[1] << "\n";
                    std::pair<std::map<int,double>::iterator,bool> ret;
                    ret = subpartsProb[coordA[0]][coordA[1]].insert( std::pair<int, double>(partFromCluster.partIds[coordB[0]][coordB[1]],1.0) );
                    if (ret.second==false) {//element exists
                        ret.first->second+=1;
                    }
                    if (part.partIds[coordA[0]][coordA[1]]>=0&&partFromCluster.partIds[coordB[0]][coordB[1]]>=0){
                        Vec4 posPoint(partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(0,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(1,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(2,0),1.0);
                        //Mat34 offset(partFromCluster.offsets[coordB[0]][coordB[1]]);
                        //std::cout << "before trans " << posPoint.transpose() << "\n";
                        posPoint = estTrans*posPoint;
                        //std::cout << "after trans " << posPoint.transpose() << "\n";
                        Vec3 norm3(partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(3,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(4,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(5,0));
                        //std::cout << "norm before trans " << norm3.transpose() << "\n";
                        norm3 = estTrans.rotation()*norm3;
                        //std::cout << "norm after trans " << norm3.transpose() << "\n";
                        Vec6 newPosNorm;
                        newPosNorm(0)=posPoint(0); newPosNorm(1)=posPoint(1); newPosNorm(2)=posPoint(2);
                        newPosNorm(3)=norm3(0); newPosNorm(4)=norm3(1); newPosNorm(5)=norm3(2);
                        subpartsPos[coordA[0]][coordA[1]].mean+=newPosNorm;
                        subpartsNo[coordA[0]][coordA[1]]++;
                    }
                    idx++;
                    //getchar();
                }
                /// for central subpart
                std::pair<std::map<int,double>::iterator,bool> ret;
                ret = subpartsProb[1][1].insert( std::pair<int, double>(partFromCluster.partIds[1][1],1.0) );
                if (ret.second==false) {//element exists
                    ret.first->second+=1;
                }
                if (partFromCluster.partIds[1][1]>=0){
                    Vec4 posPoint(partFromCluster.partsPosNorm[1][1].mean(0,0), partFromCluster.partsPosNorm[1][1].mean(1,0), partFromCluster.partsPosNorm[1][1].mean(2,0),1.0);
                    posPoint = estTrans*posPoint;
                    Vec3 norm3(partFromCluster.partsPosNorm[1][1].mean(3,0), partFromCluster.partsPosNorm[1][1].mean(4,0), partFromCluster.partsPosNorm[1][1].mean(5,0));
                    norm3 = estTrans.rotation()*norm3;
                    Vec6 newPosNorm;
                    newPosNorm(0)=posPoint(0); newPosNorm(1)=posPoint(1); newPosNorm(2)=posPoint(2);
                    newPosNorm(3)=norm3(0); newPosNorm(4)=norm3(1); newPosNorm(5)=norm3(2);
                    subpartsPos[1][1].mean+=newPosNorm;
                    subpartsNo[1][1]++;
                }
            }
            for (int i=0;i<3;i++){
                for (int j=0;j<3;j++){
                    if (part.partIds[i][j]>=0){
                        subpartsPos[i][j].mean/=double(subpartsNo[i][j]);
                        subpartsPos[i][j].mean.block<3,1>(3,0).normalized();
                    }
                    for (auto & subpartProb : subpartsProb[i][j]){
                        subpartProb.second/=double(part.group.size());
                    }
                    subpartsPos[i][j].covariance.setZero();
                }
            }
            part.subpartsProb=subpartsProb;
            //compute covariance matrix
            int partNo=0;
            for (auto& partFromCluster : part.group){
                int idx = optRot[partNo].first;
                for (size_t j=0;j<pointCorrespondence.size();j++){
                    Mat34 estTrans = optRot[partNo].second;
                    int coordA[2]={pointCorrespondence[j].first, pointCorrespondence[j].second};//partA is not rotated
                    int coordB[2]={pointCorrespondence[idx%(pointCorrespondence.size())].first, pointCorrespondence[idx%(pointCorrespondence.size())].second};//partA is not rotated
                    if (partFromCluster.partIds[coordA[0]][coordA[1]]>=0&&partFromCluster.partIds[coordB[0]][coordB[1]]>=0){
                        Vec4 posPoint(partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(0,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(1,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(2,0),1.0);
                        posPoint = estTrans*posPoint;
                        Vec3 norm3(partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(3,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(4,0), partFromCluster.partsPosNorm[coordB[0]][coordB[1]].mean(5,0));
                        norm3 = estTrans.rotation()*norm3;
                        Vec6 newPosNorm;
                        newPosNorm(0)=posPoint(0); newPosNorm(1)=posPoint(1); newPosNorm(2)=posPoint(2);
                        newPosNorm(3)=norm3(0); newPosNorm(4)=norm3(1); newPosNorm(5)=norm3(2);
                        subpartsPos[coordA[0]][coordA[1]].covariance+=(newPosNorm-subpartsPos[coordA[0]][coordA[1]].mean)*(newPosNorm-subpartsPos[coordA[0]][coordA[1]].mean).transpose();
                        subpartsNo[coordA[0]][coordA[1]]++;
                    }
                    idx++;
                }
                partNo++;
            }
            for (int i=0;i<3;i++){//update part
                for (int j=0;j<3;j++){
                    if (part.partIds[i][j]>=0){
                        subpartsPos[i][j].covariance*=1.0/double(subpartsNo[i][j]);
                        if (i==1&&j==1)
                            part.partsPosNorm[i][j].mean.block<3,1>(3,0)=subpartsPos[i][j].mean.block<3,1>(3,0);
                        else
                            part.partsPosNorm[i][j].mean=subpartsPos[i][j].mean;
                        //part.partsPosNorm[i][j].mean.block<3,1>(0,0)=subpartsPos[i][j].mean.block<3,1>(0,0);
                        if (subpartsPos[i][j].covariance.isZero(0)){
                            subpartsPos[i][j].covariance = Mat66::Identity()*std::numeric_limits<double>::epsilon();
                        }
                        part.partsPosNorm[i][j].covariance=subpartsPos[i][j].covariance;
                        //part.offsets[i][j]=Mat34::Identity();
                    }
                }
            }
        }
    }
}

/// compute statisticts (results are written to representative parts)
void Hierarchy::computeStats(void){
    for (int layerNo = 0; layerNo<(int)viewDependentLayers.size();layerNo++){//compute mean position and probability of id
        computeVDStats(layerNo);
    }
}

/// reconstruct part
int Hierarchy::reconstructPart(ViewDependentPart& _part, size_t layerNo) const{
    if (layerNo>=viewDependentLayers.size()){
        throw std::runtime_error("Compute stats: wrong layer no \n");
    }
    else{
        double maxFit = std::numeric_limits<double>::min();
        int maxId=0;
        int partId=0;
        int maxRotId=0;
        Mat34 maxTransform;
        for (const auto& part : viewDependentLayers[layerNo]){
            if (part.isComplete()){
                int rotId; Mat34 estimatedTransform;
                double fit=part.distanceStats(_part, viewDependentLayers[0], viewDependentLayers[1], rotId, estimatedTransform);
                //std::cout << "fit " << fit << " rotId " << rotId << " estTrans \n" << estimatedTransform.matrix() << "\n";
                //getchar();
                //std::cout << "fit " << fit << " fit2 " << _part.distanceStats(part, rotId, estima) << "\n";
                if (fit>maxFit){
                    maxFit=fit;
                    maxId=partId;
                    maxRotId=rotId;
                    maxTransform = estimatedTransform;
                }
            }
            partId++;
        }
        //restore occluded subparts

        if (layerNo==1){
            /*std::cout << "before restoration:\n";
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << _part.partIds[i][j] << ", ";
                }
                std::cout << "\n";
            }
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << _part.partsPosNorm[i][j].mean.block<3,1>(0,0).transpose() << ", ";
                }
                std::cout << "\n";
            }
            std::cout << "most similar part:\n";
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << viewDependentLayers[layerNo][maxId].partIds[i][j] << ", ";
                }
                std::cout << "\n";
            }
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << viewDependentLayers[layerNo][maxId].partsPosNorm[i][j].mean.block<3,1>(0,0).transpose() << ", ";
                }
                std::cout << "\n";
            }*/
        }
        _part.restoreOccluded(viewDependentLayers[layerNo][maxId], maxRotId, maxTransform.inverse());
        /*if (layerNo==1){
            std::cout << "max roty id " << maxRotId << " transform \n" << maxTransform.matrix() << "\n";
            std::cout << "after restoration:\n";
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << _part.partIds[i][j] << ", ";
                }
                std::cout << "\n";
            }
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    std::cout << _part.partsPosNorm[i][j].mean.block<3,1>(0,0).transpose() << ", ";
                }
                std::cout << "\n";
            }
            Vec4 pp(0,0.005,-0.001,1);
            std::cout << "test " << (maxTransform*pp).transpose() << "\n";
            std::cout << "test2 " << (maxTransform.inverse()*pp).transpose() << "\n";
            //getchar();
        }*/
        return maxId;
    }
    return -1;
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
