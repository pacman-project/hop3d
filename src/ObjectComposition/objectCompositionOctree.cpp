#include "hop3d/ObjectComposition/objectCompositionOctree.h"
#include <ctime>
#include "hop3d/ImageFilter/normalImageFilter.h"

using namespace hop3d;

/// A single instance of object composition
ObjectCompositionOctree::Ptr composition;

int ObjectCompositionOctree::partRealisationsCounter = 0;

ObjectCompositionOctree::ObjectCompositionOctree(void) : ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE) {
}

/// Construction
ObjectCompositionOctree::ObjectCompositionOctree(std::string _config) :
        ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE), config(_config) {
    octrees.resize(4);
    for (int layerNo=0;layerNo<4;layerNo++){
        octrees[layerNo].resize(3);
        for (int overlapNo=0;overlapNo<3;overlapNo++)
            octrees[layerNo][overlapNo].reset(new Octree<ViewIndependentPart>(config.voxelsNo/(int)pow(2,layerNo))); // cellSize0
    }
    octreeGrid.reset(new Octree<hop3d::PointCloud>(config.voxelsNoGrid));
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
    octrees.resize(4);
    for (int layerNo=0;layerNo<4;layerNo++){
        octrees[layerNo].resize(3);
        for (int overlapNo=0;overlapNo<3;overlapNo++)
            octrees[layerNo][overlapNo].reset(new Octree<ViewIndependentPart>(config.voxelsNo/(int)pow(2,layerNo))); // cellSize0
    }
    octreeGrid.reset(new Octree<hop3d::PointCloud>(config.voxelsNoGrid));
}

///config class constructor
ObjectCompositionOctree::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load Object Composition octree config file: " + filename);
    tinyxml2::XMLElement * group = config.FirstChildElement( "ObjectComposition" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load Object Composition octree parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryDoubleAttribute("voxelSize", &voxelSize);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("voxelsNo", &voxelsNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("minPatchesNo", &minPatchesNo);
    group->FirstChildElement( "pointCloudGrid" )->QueryDoubleAttribute("maxAngleGrid", &maxAngleGrid);
    group->FirstChildElement( "pointCloudGrid" )->QueryIntAttribute("voxelsNoGrid", &voxelsNoGrid);
    group->FirstChildElement( "pointCloudGrid" )->QueryDoubleAttribute("voxelSizeGrid", &voxelSizeGrid);

    std::string GICPConfig = (group->FirstChildElement( "GICP" )->Attribute( "configFilename" ));
    size_t found = configFilename.find_last_of("/\\");
    std::string prefix = configFilename.substr(0,found+1);
    GICPConfig = prefix+GICPConfig;
    tinyxml2::XMLDocument configGICPxml;
    configGICPxml.LoadFile(GICPConfig.c_str());
    if (configGICPxml.ErrorID())
        throw std::runtime_error("unable to load Object Composition octree config file: " + filename);
    tinyxml2::XMLElement * groupGICP = configGICPxml.FirstChildElement( "GICP" );

    groupGICP->QueryIntAttribute("verbose", &configGICP.verbose);
    groupGICP->QueryIntAttribute("guessesNo", &configGICP.guessesNo);
    groupGICP->QueryIntAttribute("maxIterations", &configGICP.maxIterations);
    groupGICP->QueryDoubleAttribute("transformationEpsilon", &configGICP.transformationEpsilon);
    groupGICP->QueryDoubleAttribute("EuclideanFitnessEpsilon", &configGICP.EuclideanFitnessEpsilon);
    groupGICP->QueryDoubleAttribute("correspondenceDist", &configGICP.correspondenceDist);
    groupGICP->QueryDoubleAttribute("alphaMin", &configGICP.alpha.first);
    groupGICP->QueryDoubleAttribute("alphaMax", &configGICP.alpha.second);
    groupGICP->QueryDoubleAttribute("betaMin", &configGICP.beta.first);
    groupGICP->QueryDoubleAttribute("betaMax", &configGICP.beta.second);
    groupGICP->QueryDoubleAttribute("gammaMin", &configGICP.gamma.first);
    groupGICP->QueryDoubleAttribute("gammaMax", &configGICP.gamma.second);
}

/// update composition from octets (words from last view-independent layer's vocabulary)
/*void ObjectCompositionOctree::update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose){
    for (auto & part : parts){
        if (config.verbose==1){
            std::cout << "update octree, part.id: " << part.id << " layer id" << part.layerId << "\n";
        }
        Mat34 partPosition(Mat34::Identity());
        partPosition.translation() = part.locationEucl;
        partPosition = cameraPose * partPosition*part.offset;
        int x,y,z;
        toCoordinate(partPosition(0,3),x, layerNo);
        toCoordinate(partPosition(1,3),y, layerNo);
        toCoordinate(partPosition(2,3),z, layerNo);
        if (config.verbose==1){
            std::cout << "Update octree, xyz: " << x << ", " << y << ", " << z << "\n";
        }
            //std::cout << "part.id " << part.id << "\n";
            //std::cout << "partPosition: \n" << partPosition.matrix() << "\n";
            //getchar();
        ViewIndependentPart::Part3D viewIndependentPart(partPosition, part.id);
        (*octrees[layerNo])(x,y,z).pose = partPosition;
        (*octrees[layerNo])(x,y,z).parts.push_back(viewIndependentPart);
        (*octrees[layerNo])(x,y,z).layerId=4;
        (*octrees[layerNo])(x,y,z).id=part.id;//its temporary id only
    }
}*/

/// update composition from octets (words from last view-independent layer's vocabulary)
void ObjectCompositionOctree::updatePCLGrid(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose){
    for (auto & part : parts){
        if (config.verbose==1){
            std::cout << "update octree, part.id: " << part.id << " layer id" << part.layerId << "\n";
        }
        Mat34 partPosition(Mat34::Identity());
        partPosition.translation() = part.locationEucl;
        partPosition = cameraPose * partPosition*part.offset;
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                if (part.partIds[i][j]>0){
                    PointNormal pointNorm;
                    Vec4 position(part.partsPosNorm[i][j].mean(0),part.partsPosNorm[i][j].mean(1),part.partsPosNorm[i][j].mean(2),1.0);
                    if (i==1&&j==1)
                        position = Vec4(0,0,0,1.0);
                    pointNorm.position = (partPosition*position).block<3,1>(0,0);
                    pointNorm.normal = partPosition.rotation()*part.partsPosNorm[i][j].mean.block<3,1>(3,0);
                    int x,y,z;
                    toCoordinatePCLGrid(pointNorm.position(0),x);
                    toCoordinatePCLGrid(pointNorm.position(1),y);
                    toCoordinatePCLGrid(pointNorm.position(2),z);
                    if (config.verbose==1){
                        std::cout << "(" << pointNorm.position(0) << ", " << pointNorm.position(1) << ", " << pointNorm.position(2) << ") Update octree, xyz: " << x << ", " << y << ", " << z << "\n";
                    }
                    if (x<config.voxelsNoGrid&&y<config.voxelsNoGrid&&z<config.voxelsNoGrid)
                        (*octreeGrid)(x,y,z).push_back(pointNorm);
                    else{
                        if (config.verbose>0){
                            std::cout << "Warning1: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  0 << "\n";
                            getchar();
                        }
                    }
                    toCoordinate(pointNorm.position(0),x, 0); toCoordinate(pointNorm.position(1),y,0); toCoordinate(pointNorm.position(2),z,0);
                    for (int overlapNo=0;overlapNo<3;overlapNo++){
                        if (x<(config.voxelsNo/(int)pow(2,0))&&y<(config.voxelsNo/(int)pow(2,0))&&z<(config.voxelsNo/(int)pow(2,0)))
                            (*octrees[0][overlapNo])(x,y,z).incomingIds.insert(part.id);
                        else{
                            if (config.verbose>0){
                                std::cout << "Warning2: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  0 << "\n";
                                getchar();
                            }
                        }
                    }
                }
            }
        }
    }
}

/// filter voxel grid
void ObjectCompositionOctree::filterPCLGrid(int destLayerNo){
    for (int idX=0; idX<(*octreeGrid).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octreeGrid).size(); idY++){
            for (int idZ=0; idZ<(*octreeGrid).size(); idZ++){
                if ((*octreeGrid).at(idX,idY,idZ).size()>0){//compute mean point and normal
                    std::vector<hop3d::PointCloud> groups;
                    hop3d::PointCloud means;
                    //std::cout << idX << ", " << idY << ", " << idZ << " = " << (*octreeGrid).at(idX,idY,idZ).size() << " before\n";
                    //for (auto& point : (*octreeGrid)(idX,idY,idZ)){
                    //    std::cout << "element " << point.position.transpose() << "     " << point.normal.transpose() << "\n";
                    //}
                    for (auto& pointNorm : (*octreeGrid).at(idX,idY,idZ)){
                        if (groups.size()==0){// first element
                            groups.resize(1);
                            groups[0].push_back(pointNorm);
                            means.push_back(pointNorm);
                        }
                        else{
                            int groupNo=-1;
                            int iterNo=0;
                            double minAngle = std::numeric_limits<double>::max();
                            for (auto point : means){//compute angle between vectors and add select group
                                double angle = fabs(acos(point.normal.adjoint()*pointNorm.normal));
                                if (angle<config.maxAngleGrid&&angle<minAngle){
                                    minAngle=angle;
                                    groupNo=iterNo;
                                }
                                iterNo++;
                            }
                            if (groupNo==-1){
                                groups.resize(groups.size()+1);
                                groups[groups.size()-1].push_back(pointNorm);
                                means.push_back(pointNorm);
                            }
                            else{
                                groups[groupNo].push_back(pointNorm);
                                means[groupNo] = computeMeanPosNorm(groups[groupNo]);
                            }
                        }
                    }
                    if (idX<config.voxelsNoGrid&&idY<config.voxelsNoGrid&&idZ<config.voxelsNoGrid)
                        (*octreeGrid)(idX,idY,idZ) = means;
                    else{
                        if (config.verbose>0){
                            std::cout << "Warning3: octree index out of range. " << idX << ", " << idY << ", " << idZ << ", " << " layer " <<  0 << "\n";
                            getchar();
                        }
                    }
                    //means = (*octreeGrid).at(idX,idY,idZ);
                    for (auto& point : means){//update parts octree
                        int x,y,z;
                        toCoordinate(point.position(0),x,destLayerNo); toCoordinate(point.position(1),y,destLayerNo); toCoordinate(point.position(2),z,destLayerNo);
                        for (int overlapNo=0;overlapNo<3;overlapNo++){
                            if (x<(config.voxelsNo/(int)pow(2,destLayerNo))&&y<(config.voxelsNo/(int)pow(2,destLayerNo))&&z<(config.voxelsNo/(int)pow(2,destLayerNo)))
                                (*octrees[destLayerNo][overlapNo])(x,y,z).cloud.push_back(point);
                            else{
                                if (config.verbose>0){
                                    std::cout << "Warning4: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  destLayerNo << "\n";
                                    getchar();
                                }
                            }
                        }
                    }
                    //std::cout << idX << ", " << idY << ", " << idZ << " = " << (*octreeGrid).at(idX,idY,idZ).size() << " after\n";
                    //for (auto& point : (*octreeGrid)(idX,idY,idZ)){
                    //    std::cout << "element " << point.position.transpose() << "     " << point.normal.transpose() << "\n";
                    //}
                    //getchar();
                }
            }
        }
    }
}

/// compute mean value of normal and position
PointNormal ObjectCompositionOctree::computeMeanPosNorm(PointCloud cloud){
    Vec3 meanPos(0,0,0);
    Vec3 meanNorm(0,0,0);
    for (auto& point : cloud){
        meanPos+=point.position;
        meanNorm+=point.normal;
    }
    PointNormal mean;
    meanNorm/=double(cloud.size());
    meanNorm.normalize();
    meanPos/=double(cloud.size());
    mean.normal = meanNorm;
    mean.position = meanPos;
    return mean;
}

/// upodate voxel poses using new vocabulary
void ObjectCompositionOctree::updateVoxelsPose(int layerNo, const std::vector<ViewIndependentPart>& vocabulary){
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        for (int idX=0; idX<(*octrees[layerNo][overlapNo]).size(); idX++){///to do z-slicing
            for (int idY=0; idY<(*octrees[layerNo][overlapNo]).size(); idY++){
                for (int idZ=0; idZ<(*octrees[layerNo][overlapNo]).size(); idZ++){
                    if ((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).id>=0){
                        int wordId=0;
                        double minDist=std::numeric_limits<double>::max();
                        for (auto& word: vocabulary){
                            Mat34 estTransform;
                            double fitness = pow(1+fabs(double(word.cloud.size())-double((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).cloud.size())),2.0)*ViewIndependentPart::distanceGICP(word,(*octrees[layerNo][overlapNo])(idX,idY,idZ),config.configGICP,estTransform);
                            if (fitness<minDist){//find min distance
                                minDist=fitness;
                                (*octrees[layerNo][overlapNo])(idX,idY,idZ).id = wordId;
                                (*octrees[layerNo][overlapNo])(idX,idY,idZ).offset=estTransform;
                                (*octrees[layerNo][overlapNo])(idX,idY,idZ).realisationId = partRealisationsCounter;
                                partRealisationsCounter++;
                            }
                            wordId++;
                        }
                    }
                }
            }
        }
    }
}

/// get set of ids for the given input point
void ObjectCompositionOctree::getPartsIds(const Vec3& point, int overlapNo, std::vector<int>& ids) const{
    int x,y,z;
    for (int layNo=0;layNo<3;layNo++){
        toCoordinate(point(0),x, layNo);    toCoordinate(point(1),y, layNo);    toCoordinate(point(2),z, layNo);
        //std::cout << "coordinate " << x << " " << y << " " << z << "\n";
        if ((x-overlapNo)>-1&&(y-overlapNo)>-1&&(z-overlapNo)>-1){
            if ((x-overlapNo)%3==0) x+=1; else if ((x-overlapNo)%3==2) x-=1;
            if ((y-overlapNo)%3==0) y+=1; else if ((y-overlapNo)%3==2) y-=1;
            if ((z-overlapNo)%3==0) z+=1; else if ((z-overlapNo)%3==2) z-=1;
            if (x<(config.voxelsNo/(int)pow(2,layNo))&&y<(config.voxelsNo/(int)pow(2,layNo))&&z<(config.voxelsNo/(int)pow(2,layNo))){
                ids.push_back((*octrees[layNo][overlapNo]).at(x,y,z).id);//4th layer
            }
            else{
                if (config.verbose>0){
                    std::cout << "Warning5: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  layNo << "\n";
                    getchar();
                }
            }
        }
    }
}

/// get parts realisations
void ObjectCompositionOctree::getPartsRealisation(int layerNo, int overlapNo, std::vector<ViewIndependentPart::Part3D>& partsViewTmp) const{
    for (int idX=0; idX<(*octrees[layerNo][overlapNo]).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octrees[layerNo][overlapNo]).size(); idY++){
            for (int idZ=0; idZ<(*octrees[layerNo][overlapNo]).size(); idZ++){
                if ((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).id>=0){
                    ViewIndependentPart::Part3D part;
                    part.id = (*octrees[layerNo][overlapNo]).at(idX,idY,idZ).id;
                    part.realisationId = (*octrees[layerNo][overlapNo]).at(idX,idY,idZ).realisationId;
                    part.pose = (*octrees[layerNo][overlapNo])(idX,idY,idZ).pose * (*octrees[layerNo][overlapNo])(idX,idY,idZ).offset;
                    partsViewTmp.push_back(part);
                }
            }
        }
    }
}

/// get realisations ids
void ObjectCompositionOctree::getRealisationsIds(const Vec3& point, int overlapNo, std::vector<int>& ids) const{
    int x,y,z;
    for (int layNo=0;layNo<3;layNo++){
        toCoordinate(point(0),x, layNo);    toCoordinate(point(1),y, layNo);    toCoordinate(point(2),z, layNo);
        //std::cout << "coordinate " << x << " " << y << " " << z << "\n";
        if ((x-overlapNo)>-1&&(y-overlapNo)>-1&&(z-overlapNo)>-1){
            if ((x-overlapNo)%3==0) x+=1; else if ((x-overlapNo)%3==2) x-=1;
            if ((y-overlapNo)%3==0) y+=1; else if ((y-overlapNo)%3==2) y-=1;
            if ((z-overlapNo)%3==0) z+=1; else if ((z-overlapNo)%3==2) z-=1;
            if (x<(config.voxelsNo/(int)pow(2,layNo))&&y<(config.voxelsNo/(int)pow(2,layNo))&&z<(config.voxelsNo/(int)pow(2,layNo))){
                if ((*octrees[layNo][overlapNo]).at(x,y,z).realisationId>=0)
                    ids.push_back((*octrees[layNo][overlapNo]).at(x,y,z).realisationId);//4th layer
                else
                    ids.push_back(-2);
            }
            else{
                if (config.verbose>0){
                    std::cout << "Warning5: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  layNo << "\n";
                    getchar();
                }
            }
        }
    }
}

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinate(double pos, int& coord, int layerNo) const{
    coord = int(pos/(pow(3.0,layerNo)*config.voxelSize))+(int)(config.voxelsNo/(2*pow(3.0,layerNo)));
}

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinate(int coordPrevLayer, int& coord, int overlapNo) const{
    if (coordPrevLayer<overlapNo)
        throw std::runtime_error("wrong octree coordinate\n");
    coord = (coordPrevLayer-overlapNo)/3;
}

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinate(double pos, int& coord, int layerNo, int overlapNo) const{
    coord = int((pos-(overlapNo*pow(3.0,layerNo-1)*config.voxelSize))/(pow(3.0,layerNo)*config.voxelSize))+(int)(config.voxelsNo/(2*pow(3.0,layerNo)));
}

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinatePCLGrid(double pos, int& coord) const{
    coord = int(pos/(config.voxelSizeGrid))+(int)(config.voxelsNoGrid/2);
}

/// convert octree coordinates to global coordinates
void ObjectCompositionOctree::fromCoordinate(int coord, double& pos, int layerNo) const{
    pos = (pow(3.0,layerNo)*config.voxelSize)*(coord-config.voxelsNo/(2*pow(3.0,layerNo)))+((pow(3.0,layerNo)*config.voxelSize)/2.0);
}

/// convert octree coordinates to global coordinates
void ObjectCompositionOctree::fromCoordinate(int coord, double& pos, int layerNo, int overlapNo) const{
    pos = (overlapNo*pow(3.0,layerNo-1)*config.voxelSize)+(pow(3.0,layerNo)*config.voxelSize)*(coord-config.voxelsNo/(2*pow(3.0,layerNo)))+((pow(3.0,layerNo)*config.voxelSize)/2.0);
}

/// convert octree coordinates to global coordinates
void ObjectCompositionOctree::fromCoordinatePCLGrid(int coord, double& pos) const{
    pos = (config.voxelSizeGrid)*(coord-config.voxelsNoGrid/(2))+(config.voxelSizeGrid/2.0);
}

/// update ids in the octree using new vocabulary
/*void ObjectCompositionOctree::updateIds(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy){
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        for (int idX=0; idX<(*octrees[layerNo][overlapNo]).size(); idX++){///to do z-slicing
            for (int idY=0; idY<(*octrees[layerNo][overlapNo]).size(); idY++){
                for (int idZ=0; idZ<(*octrees[layerNo][overlapNo]).size(); idZ++){
                    if ((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).id>=0){
                        //find part in the vocabulary
                       // int idOld = (*octrees[layerNo])(idX,idY,idZ).id;
                        (*octrees[layerNo][overlapNo])(idX,idY,idZ).id = findIdInVocabulary((*octrees[layerNo][overlapNo]).at(idX,idY,idZ), vocabulary);
                        if ((*octrees[layerNo][overlapNo])(idX,idY,idZ).layerId==5){//compute distance from centroid
                            Mat34 off;
                            ViewIndependentPart::distance(hierarchy.viewIndependentLayers[1][(*octrees[layerNo][overlapNo])(idX,idY,idZ).id], (*octrees[layerNo][overlapNo])(idX,idY,idZ), off);
                            (*octrees[layerNo][overlapNo])(idX,idY,idZ).offset = off;
                            //std::cout << "old id " << idOld << ", new id " << (*octrees[layerNo])(idX,idY,idZ).id << "\n";
                            //std::cout << "offset \n" << (*octrees[layerNo])(idX,idY,idZ).offset.matrix() << "\n";
                            //(*octrees[layerNo])(idX,idY,idZ).offset=Mat34::Identity();
                            //getchar();
                        }
                        if ((*octrees[layerNo][overlapNo])(idX,idY,idZ).layerId==6){//compute distance from centroid
                            Mat34 off;
                            ViewIndependentPart::distance(hierarchy.viewIndependentLayers[2][(*octrees[layerNo][overlapNo])(idX,idY,idZ).id], (*octrees[layerNo][overlapNo])(idX,idY,idZ), hierarchy.viewIndependentLayers[1], off);
                            (*octrees[layerNo][overlapNo])(idX,idY,idZ).offset = off;
                            //std::cout << "old id " << idOld << ", new id " << (*octrees[layerNo])(idX,idY,idZ).id << "\n";
                            //std::cout << "offset \n" << (*octrees[layerNo])(idX,idY,idZ).offset.matrix() << "\n";
                            //(*octrees[layerNo])(idX,idY,idZ).offset=Mat34::Identity();
                            //getchar();
                        }
                    }
                }
            }
        }
    }
}*/

/// find part in the vocabulary and return new id
int ObjectCompositionOctree::findIdInVocabulary(const ViewIndependentPart& part, const std::vector<ViewIndependentPart>& vocabulary){
    int partId=0;
    for (auto & vpart : vocabulary){
        for (auto & gpart : vpart.group){
            if (part.partIds == gpart.partIds){
                return partId;
            }
        }
        partId++;
    }
    std::cout << "Could not find part in the vocabulary!\n";
    getchar();
    return -1;
}

/// get octree in layer layerNo
void ObjectCompositionOctree::getParts(int layerNo, std::vector<ViewIndependentPart>& parts){
    parts.clear();
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        for (int idX=0; idX<(*octrees[layerNo][overlapNo]).size(); idX++){///to do z-slicing
            for (int idY=0; idY<(*octrees[layerNo][overlapNo]).size(); idY++){
                for (int idZ=0; idZ<(*octrees[layerNo][overlapNo]).size(); idZ++){
                    if ((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).id>=0){
                        parts.push_back((*octrees[layerNo][overlapNo]).at(idX,idY,idZ));
                    }
                }
            }
        }
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
void ObjectCompositionOctree::getClusters(int layerNo, std::vector< std::set<int>>& clusters){
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        for (int idX=0; idX<(*octrees[layerNo][overlapNo]).size(); idX++){///to do z-slicing
            for (int idY=0; idY<(*octrees[layerNo][overlapNo]).size(); idY++){
                for (int idZ=0; idZ<(*octrees[layerNo][overlapNo]).size(); idZ++){
                    if ((*octrees[layerNo][overlapNo]).at(idX,idY,idZ).parts.size()>0){
                        std::set<int> partIds1, partIds2;
                        int iterNo=0;
                        Vec3 norm1;
                        for (auto & part : (*octrees[layerNo][overlapNo])(idX,idY,idZ).parts){
                            if (iterNo>0){
                                Vec3 norm2 = part.pose.matrix().block(0,2,3,1);
                                //compute angle between normals
                                double dp = norm1.adjoint()*norm2;
                                if (fabs(acos(dp))>config.maxAngleGrid)// if angle between normals is bigger than threshold add to another cluster
                                    partIds2.insert(part.id);
                                else
                                    partIds1.insert(part.id);
                            }
                            else {
                                partIds1.insert(part.id);
                                norm1=part.pose.matrix().block(0,2,3,1);
                            }
                            iterNo++;
                        }
                        if (partIds1.size()>0)
                            clusters.push_back(partIds1);
                        if (partIds2.size()>0)
                            clusters.push_back(partIds2);
                    }
                }
            }
        }
    }
}

/// compute rotation matrix from normal vector ('y' axis is vetical)
/*void ObjectCompositionOctree::normal2rot(const Vec3& normal, Mat33& rot){
    Vec3 y(0,1,0); Vec3 x;
    Vec3 _normal(normal);
    x = y.cross(_normal);
    DepthSensorModel::normalizeVector(x);
    y = _normal.cross(x);
    DepthSensorModel::normalizeVector(y);
    rot.block(0,0,3,1) = x;
    rot.block(0,1,3,1) = y;
    rot.block(0,2,3,1) = _normal;
}*/

/// get all patches from the voxel grid
int ObjectCompositionOctree::getCorrespondingPatches(hop3d::PointCloud& patches, int centerX, int centerY, int centerZ, int coeff){
    patches.clear();
    for (int i=-int(coeff/2); i<int(coeff/2);i++){
        for (int j=-int(coeff/2); j<int(coeff/2);j++){
            for (int k=-int(coeff/2); k<int(coeff/2);k++){
                if ((*octreeGrid).at(centerX+i, centerY+j, centerZ+k).size()>0){
                    std::cout << "get grid " << centerX << " " << centerY << " " << centerZ << "\n";
                    patches.insert(patches.end(),(*octreeGrid).at(centerX+i, centerY+j, centerZ+k).begin(), (*octreeGrid).at(centerX+i, centerY+j, centerZ+k).end());
                }
            }
        }
    }
    return (int)patches.size();
}

/// assign neighbouring parts to new part
int ObjectCompositionOctree::createFirstLayerPart(ViewIndependentPart& newPart, int overlapNo, int x, int y, int z){
    int partsNo=0;
    //||||||||||
    //|  |  |  |
    //|||||||||||||
    //|   |   |   |
    double center[3];
    fromCoordinate(x,center[0],0); fromCoordinate(y,center[1],0); fromCoordinate(z,center[2],0);
    for (int i=-1; i<2;i++){
        for (int j=-1; j<2;j++){
            for (int k=-1; k<2;k++){
                if (x+i<(config.voxelsNo/(int)pow(2,0))&&y+j<(config.voxelsNo/(int)pow(2,0))&&z+k<(config.voxelsNo/(int)pow(2,0))){
                    if ((*octrees[0][overlapNo]).at(x+i,y+j,z+k).cloud.size()>0){
                        for (auto& patch : (*octrees[0][overlapNo])(x+i,y+j,z+k).cloud){
                            for (int coord = 0; coord<3; coord++)
                                patch.position(coord)-=center[coord];
                            newPart.cloud.push_back(patch);
                            partsNo++;
                        }
                        newPart.incomingIds.insert((*octrees[0][overlapNo]).at(x+i,y+j,z+k).incomingIds.begin(), (*octrees[0][overlapNo]).at(x+i,y+j,z+k).incomingIds.end());
                        newPart.partIds[i+1][j+1][k+1] = 1;
                    }
                    else
                        newPart.partIds[i+1][j+1][k+1] = -1;
                }
                else{
                    if (config.verbose>0){
                        std::cout << "Warning6: octree index out of range. " << x+i << ", " << y+j << ", " << z+k << ", " << " layer " <<  0 << "\n";
                        getchar();
                    }
                }
            }
        }
    }
    if (partsNo>0){
        Mat34 partPosition(Mat34::Identity());
        partPosition(0,3) = center[0]; partPosition(1,3) = center[1]; partPosition(2,3) = center[2];
        newPart.pose = partPosition;
    }
    return partsNo;
}

/// assign neighbouring parts to new part
int ObjectCompositionOctree::createNextLayerPart(ViewIndependentPart& newPart, int layerNo, int overlapNo, int x, int y, int z){
    int partsNo=0;
    //||||||||||
    //|  |  |  |
    //|||||||||||||
    //|   |   |   |
    double center[3];
    fromCoordinate(x,center[0],layerNo); fromCoordinate(y,center[1],layerNo); fromCoordinate(z,center[2],layerNo);
    for (int i=-1; i<2;i++){
        for (int j=-1; j<2;j++){
            for (int k=-1; k<2;k++){
                if (x+i<(config.voxelsNo/(int)pow(2,layerNo))&&y+j<(config.voxelsNo/(int)pow(2,layerNo))&&z+k<(config.voxelsNo/(int)pow(2,layerNo))){
                    if ((*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).cloud.size()>0){
                        for (auto& patch : (*octrees[layerNo][overlapNo])(x+i,y+j,z+k).cloud){
                            for (int coord = 0; coord<3; coord++)
                                patch.position(coord)-=center[coord];
                            newPart.cloud.push_back(patch);
                            partsNo++;
                        }
                        newPart.incomingIds.insert((*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).incomingIds.begin(), (*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).incomingIds.end());
                        newPart.partIds[i+1][j+1][k+1] = 1;
                    }
                    else
                        newPart.partIds[i+1][j+1][k+1] = -1;
                }
                else{
                    if (config.verbose>0){
                        std::cout << "Warning6: octree index out of range. " << x+i << ", " << y+j << ", " << z+k << ", " << " layer " <<  0 << "\n";
                        getchar();
                    }
                }
            }
        }
    }
    if (partsNo>0){
        Mat34 partPosition(Mat34::Identity());
        partPosition(0,3) = center[0]; partPosition(1,3) = center[1]; partPosition(2,3) = center[2];
        newPart.pose = partPosition;
    }
    return partsNo;
}
/// assign neighbouring parts to new part
int ObjectCompositionOctree::createNextLayerPart(const Hierarchy& hierarchy, int destLayerNo, int overlapNo, ViewIndependentPart& newPart, int x, int y, int z){
    int partsNo=0;
    double center[3];
    fromCoordinate(x,center[0],destLayerNo-1); fromCoordinate(y,center[1],destLayerNo-1); fromCoordinate(z,center[2],destLayerNo-1);
    for (int i=-1; i<2;i++){
        for (int j=-1; j<2;j++){
            for (int k=-1; k<2;k++){
                int coords[3]={x+3*i,y+3*j,z+3*k};
                if (coords[0]<(config.voxelsNo/(int)pow(2,destLayerNo-1))&&coords[1]<(config.voxelsNo/(int)pow(2,destLayerNo-1))&&coords[2]<(config.voxelsNo/(int)pow(2,destLayerNo-1))){
                    if ((*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).id>=0&&(*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).cloud.size()>0){
                        ViewIndependentPart part = hierarchy.viewIndependentLayers[destLayerNo-1][(*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).id];
                        for (const auto& patch : part.cloud){
                            PointNormal patchTmp = patch;
                            Mat34 partPose = (*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).pose;
                            Mat34 offset = (*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).offset;
                            Vec3 patchPos = partPose*offset*Vec4(patch.position(0),patch.position(1),patch.position(2),1.0).block<3,1>(0,0);
                            for (int coord = 0; coord<3; coord++)
                                patchTmp.position(coord)=patchPos(coord)-center[coord];
                            patchTmp.normal = partPose.rotation()*offset.rotation()*patchTmp.normal;
                            newPart.cloud.push_back(patchTmp);
                            partsNo++;
                        }
                        //std::cout << "add id " << (*octrees[destLayerNo-1]).at(3*(x+i)+1,3*(y+j)+1,3*(z+k)+1).id << "\n";
                        newPart.incomingIds.insert((*octrees[destLayerNo-1][overlapNo]).at(coords[0],coords[1],coords[2]).id);
                        newPart.partIds[i+1][j+1][k+1] = 1;
                    }
                    else
                        newPart.partIds[i+1][j+1][k+1] = -1;
                }
                else{
                    if (config.verbose>0){
                        std::cout << "index layer i+1 " << x << ", " << y << ", " << z << "\n";
                        std::cout << "Warning7: octree index out of range. " << (3*x)+i+1 << ", " << (3*y)+j+1 << ", " << (3*z)+k+1 << ", " << " layer " <<  destLayerNo << "overlap No " << overlapNo << "\n";
                        getchar();
                    }
                }
            }
        }
    }
    if (partsNo>0){
        Mat34 partPosition(Mat34::Identity());
        partPosition(0,3) = center[0]; partPosition(1,3) = center[1]; partPosition(2,3) = center[2];
        newPart.pose = partPosition;
    }
    return partsNo;
}

/// create first layer vocabulary from voxel grid
void ObjectCompositionOctree::createFirstLayer(std::vector<ViewIndependentPart>& vocabulary){
    int tempId=0;
    // update next layer octree
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        int wordsNo=0;
        for (int idX=1+overlapNo; idX<(*octrees[0][overlapNo]).size()-1; idX+=3){///to do z-slicing
            for (int idY=1+overlapNo; idY<(*octrees[0][overlapNo]).size()-1; idY+=3){
                for (int idZ=1+overlapNo; idZ<(*octrees[0][overlapNo]).size()-1; idZ+=3){
                    ViewIndependentPart newPart;
                    newPart.layerId=3;
                    // add neighbouring parts into structure
                    if (createFirstLayerPart(newPart, overlapNo, idX, idY, idZ)>0){
                        if (newPart.cloud.size()>(unsigned)config.minPatchesNo){
                            newPart.id = tempId;//hierarchy.interpreter.at((*octrees[destLayerNo-1]).at(idX, idY, idZ).id);
                            (*octrees[0][overlapNo])(idX, idY, idZ) = newPart;//update octree
                            vocabulary.push_back(newPart);
                            tempId++;
                            wordsNo++;
                        }
                    }
                }
            }
        }
        //std::cout << "overNo " << overlapNo << " partsNo " << wordsNo << "\n";
    }
}

/// create next layer vocabulary
void ObjectCompositionOctree::createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary){
    vocabulary.clear();
    //for no reason take the middle part. In the feature it should be max, or weighted combination
    if (destLayerNo==0){
        std::cout << "create first layer vocabulary\n";
        filterPCLGrid(0);
        createFirstLayer(vocabulary);
        octreeGrid.reset(new Octree<hop3d::PointCloud>(config.voxelsNoGrid));
        std::cout << "create first layer vocabulary finished\n";
    }
    else{
        //update PCL grid
        std::cout << "update PCL grid\n";
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            for (int idX=1+overlapNo; idX<(*octrees[destLayerNo-1][overlapNo]).size()-1; idX+=3){///to do z-slicing
                for (int idY=1+overlapNo; idY<(*octrees[destLayerNo-1][overlapNo]).size()-1; idY+=3){
                    for (int idZ=1+overlapNo; idZ<(*octrees[destLayerNo-1][overlapNo]).size()-1; idZ+=3){
                        int partId = (*octrees[destLayerNo-1][overlapNo]).at(idX, idY, idZ).id;
                        if (partId>=0){
                            ViewIndependentPart part = hierarchy.viewIndependentLayers[destLayerNo-1][(*octrees[destLayerNo-1][overlapNo]).at(idX, idY, idZ).id];
                            double center[3];
                            fromCoordinate(idX,center[0],destLayerNo-1); fromCoordinate(idY,center[1],destLayerNo-1); fromCoordinate(idZ,center[2],destLayerNo-1);
                            for (const auto& point : part.cloud){
                                int x,y,z;
                                PointNormal patchTmp = point;
                                Mat34 partPose = (*octrees[destLayerNo-1][overlapNo]).at(idX, idY, idZ).pose;
                                Mat34 offset = (*octrees[destLayerNo-1][overlapNo]).at(idX, idY, idZ).offset;
                                Vec3 patchPos = partPose*offset*Vec4(point.position(0),point.position(1),point.position(2),1.0).block<3,1>(0,0);
                                for (int coord = 0; coord<3; coord++)
                                    patchTmp.position(coord)=patchPos(coord);//-center[coord];
                                patchTmp.normal = partPose.rotation()*offset.rotation()*patchTmp.normal;
                                toCoordinatePCLGrid(patchTmp.position(0),x);
                                toCoordinatePCLGrid(patchTmp.position(1),y);
                                toCoordinatePCLGrid(patchTmp.position(2),z);
                                if (config.verbose==1){
                                    std::cout << "(" << patchTmp.position(0) << ", " << patchTmp.position(1) << ", " << patchTmp.position(2) << ") Update octree, xyz: " << x << ", " << y << ", " << z << "\n";
                                }
                                if (x<config.voxelsNoGrid&&y<config.voxelsNoGrid&&z<config.voxelsNoGrid)
                                    (*octreeGrid)(x,y,z).push_back(patchTmp);
                                else{
                                    if (config.verbose>0){
                                        std::cout << "Warning1: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  0 << "\n";
                                        getchar();
                                    }
                                }
                                toCoordinate(patchTmp.position(0),x, destLayerNo); toCoordinate(patchTmp.position(1),y,destLayerNo); toCoordinate(patchTmp.position(2),z,destLayerNo);
                                for (int overlapNo=0;overlapNo<3;overlapNo++){
                                    if (x<(config.voxelsNo/(int)pow(2,0))&&y<(config.voxelsNo/(int)pow(2,0))&&z<(config.voxelsNo/(int)pow(2,0)))
                                        (*octrees[destLayerNo][overlapNo])(x,y,z).incomingIds.insert(partId);
                                    else{
                                        if (config.verbose>0){
                                            std::cout << "Warning2: octree index out of range. " << x << ", " << y << ", " << z << ", " << " layer " <<  0 << "\n";
                                            getchar();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        std::cout << "update PCL grid finished\n";
        std::cout << "filtering PCL grid\n";
        filterPCLGrid(destLayerNo);
        int tempId=0;
        // update next layer octree
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            int wordsNo=0;
            for (int idX=1+overlapNo; idX<(*octrees[destLayerNo][overlapNo]).size()-1; idX+=3){///to do z-slicing
                for (int idY=1+overlapNo; idY<(*octrees[destLayerNo][overlapNo]).size()-1; idY+=3){
                    for (int idZ=1+overlapNo; idZ<(*octrees[destLayerNo][overlapNo]).size()-1; idZ+=3){
                        ViewIndependentPart newPart;
                        newPart.layerId=3;
                        // add neighbouring parts into structure
                        if (createNextLayerPart(newPart, destLayerNo, overlapNo, idX, idY, idZ)>0){
                            if (newPart.cloud.size()>(unsigned)config.minPatchesNo){
                                newPart.id = tempId;//hierarchy.interpreter.at((*octrees[destLayerNo-1]).at(idX, idY, idZ).id);
                                (*octrees[destLayerNo][overlapNo])(idX, idY, idZ) = newPart;//update octree
                                vocabulary.push_back(newPart);
                                tempId++;
                                wordsNo++;
                            }
                        }
                    }
                }
            }
            //std::cout << "overNo " << overlapNo << " partsNo " << wordsNo << "\n";
        }
        octreeGrid.reset(new Octree<hop3d::PointCloud>(config.voxelsNoGrid));
        std::cout << "filtering PCL grid finished\n";
        //end update pcl grid
        /*int tempId=0;
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            int wordsNo=0;
            for (int idX=4+overlapNo; idX<(*octrees[destLayerNo-1][overlapNo]).size()-1; idX+=9){///to do z-slicing
                for (int idY=4+overlapNo; idY<(*octrees[destLayerNo-1][overlapNo]).size()-1; idY+=9){
                    for (int idZ=4+overlapNo; idZ<(*octrees[destLayerNo-1][overlapNo]).size()-1; idZ+=9){
                        ViewIndependentPart newPart;
                        // add neighbouring parts into structure
                        if (createNextLayerPart(hierarchy, destLayerNo, overlapNo, newPart, idX, idY, idZ)>0){
                            if (newPart.cloud.size()>(unsigned)config.minPatchesNo){
                                newPart.id = tempId;
                                newPart.layerId=destLayerNo+3;
                                //double posPrevLay[3];
                                //fromCoordinate(idX,posPrevLay[0],destLayerNo-1); fromCoordinate(idY,posPrevLay[1],destLayerNo-1); fromCoordinate(idZ,posPrevLay[2],destLayerNo-1);
                                int newCoords[3];
                                //toCoordinate(posPrevLay[0],newCoords[0],destLayerNo); toCoordinate(posPrevLay[1],newCoords[1],destLayerNo); toCoordinate(posPrevLay[2],newCoords[2],destLayerNo);
                                toCoordinate(idX,newCoords[0],overlapNo); toCoordinate(idY,newCoords[1],overlapNo); toCoordinate(idZ,newCoords[2],overlapNo);
                                (*octrees[destLayerNo][overlapNo])(newCoords[0],newCoords[1],newCoords[2]) = newPart;//update octree
                                vocabulary.push_back(newPart);
                                tempId++;
                                wordsNo++;
                            }
                        }
                    }
                }
            }
            //std::cout << "1overNo " << overlapNo << " partsNo " << wordsNo << "\n";
        }*/
    }
}

/// assign neighbouring parts to new part
int ObjectCompositionOctree::assignPartNeighbours(ViewIndependentPart& partVoxel, const Hierarchy& hierarchy, int layerNo, int overlapNo, int x, int y, int z){
    Mat34 pose(Mat34::Identity());
    fromCoordinate(x, pose(0,3), layerNo);
    fromCoordinate(y, pose(1,3), layerNo);
    fromCoordinate(z, pose(2,3), layerNo);
    //pose(0,3) = scale*x+(scale/2.0); pose(1,3) = scale*y+(scale/2.0); pose(2,3) = scale*z+(scale/2.0);
//    if ((*octrees[layerNo]).at(x,y,z).id!=-1){
//        partVoxel.pose = (*octrees[layerNo]).at(x,y,z).pose;
    //}
    //else
        partVoxel.pose = pose;
    Mat34 partPoseInv(pose.inverse());
    int partsNo=0;
    for (int i=-1; i<2;i++){
        for (int j=-1; j<2;j++){
            for (int k=-1; k<2;k++){
                if (x+i<(config.voxelsNo/(int)pow(2,layerNo))&&y+j<(config.voxelsNo/(int)pow(2,layerNo))&&z+j<(config.voxelsNo/(int)pow(2,layerNo))){
                    if ((*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).id>=0){
                        //assign neighbouring ids
                        if (layerNo==0)
                            partVoxel.partIds[i+1][j+1][k+1] = hierarchy.interpreter.at((*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).id);
                        else
                            partVoxel.partIds[i+1][j+1][k+1] = (*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).id;
                        // assign spatial relation
                        if ((*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).id!=-1){
                            //if (i==0&&j==0&&k==0){
                            //    partVoxel.neighbourPoses[i+1][j+1][k+1] = (*octrees[layerNo]).at(x,y,z).pose;//set global pose
                            //}
                            //else {
                                partVoxel.neighbourPoses[i+1][j+1][k+1] = partPoseInv*(*octrees[layerNo][overlapNo]).at(x+i,y+j,z+k).pose;//set relative pose
                                //std::cout << "partPoseInv \n" << partPoseInv.matrix()<< "\n";
                                //std::cout << "(*octrees[layerNo]).at(x+i,y+j,z+k).parts[0].pose\n" << (*octrees[layerNo]).at(x+i,y+j,z+k).parts[0].pose.matrix() <<"\n";
                                //std::cout << "partVoxel.neighbourPoses[i+1][j+1][k+1]\n" << partVoxel.neighbourPoses[i+1][j+1][k+1].matrix() << "\n";
                                //getchar();
                            //}
                        }
                        partsNo++;
                    }
                    else
                        partVoxel.partIds[i+1][j+1][k+1] = -1;
                }
                else{
                    if (config.verbose>0){
                        std::cout << "Warning8: octree index out of range. " << x+i << ", " << y+j << ", " << z+k << ", " << " layer " <<  layerNo << "\n";
                        getchar();
                    }
                }
            }
        }
    }
    return partsNo;
}

/// set realisation counter
void ObjectCompositionOctree::setRealisationCounter(int startRealisationId){
    partRealisationsCounter = startRealisationId;
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(void) {
    composition.reset(new ObjectCompositionOctree());
    return composition.get();
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(std::string config) {
    composition.reset(new ObjectCompositionOctree(config));
    return composition.get();
}
