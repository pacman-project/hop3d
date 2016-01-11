#include "hop3d/ObjectComposition/objectCompositionOctree.h"
#include <ctime>
#include "hop3d/ImageFilter/normalImageFilter.h"

using namespace hop3d;

/// A single instance of object composition
ObjectCompositionOctree::Ptr composition;

ObjectCompositionOctree::ObjectCompositionOctree(void) : ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE) {
}

/// Construction
ObjectCompositionOctree::ObjectCompositionOctree(std::string _config) :
        ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE), config(_config) {
    octrees.resize(4);
    octrees[0].reset(new Octree<ViewIndependentPart>(config.voxelsNo)); // cellSize0
    octrees[1].reset(new Octree<ViewIndependentPart>(config.voxelsNo/2)); // cellSize1=cellSize0*3;
    octrees[2].reset(new Octree<ViewIndependentPart>(config.voxelsNo/4)); // cellSize2=cellSize1*3=cellSize0*9;
    octrees[3].reset(new Octree<ViewIndependentPart>(config.voxelsNo/8)); // cellSize2=cellSize1*3=cellSize0*9;
    octreeGrid.reset(new Octree<hop3d::PointCloud>(config.voxelsNoGrid));
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
    octrees.resize(4);
    octrees[0].reset(new Octree<ViewIndependentPart>(2)); // cellSize0
    octrees[1].reset(new Octree<ViewIndependentPart>(4)); // cellSize1=cellSize0*3;
    octrees[2].reset(new Octree<ViewIndependentPart>(8)); // cellSize2=cellSize1*3=cellSize0*9;
    octrees[3].reset(new Octree<ViewIndependentPart>(16)); // cellSize2=cellSize1*3=cellSize0*9;
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

    group->FirstChildElement( "GICP" )->QueryIntAttribute("verbose", &configGICP.verbose);
    group->FirstChildElement( "GICP" )->QueryIntAttribute("guessesNo", &configGICP.guessesNo);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("correspondenceDist", &configGICP.correspondenceDist);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("alphaMin", &configGICP.alpha.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("alphaMax", &configGICP.alpha.second);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("betaMin", &configGICP.beta.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("betaMax", &configGICP.beta.second);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("gammaMin", &configGICP.gamma.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("gammaMax", &configGICP.gamma.second);
}

/// update composition from octets (words from last view-independent layer's vocabulary)
void ObjectCompositionOctree::update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, Hierarchy& hierarchy){
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
}

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
                if (part.partIds[i][j]!=-1){
                    PointNormal pointNorm;
                    Vec4 position(part.partsPosNorm[i][j].mean(0),part.partsPosNorm[i][j].mean(1),part.partsPosNorm[i][j].mean(2),1.0);
                    if (i==1&&j==1)
                        position = Vec4(0,0,0,1.0);
                    pointNorm.position = (partPosition*position).block<3,1>(0,0);
                    Vec3 normal(part.partsPosNorm[i][j].mean(3),part.partsPosNorm[i][j].mean(4),part.partsPosNorm[i][j].mean(5));
                    pointNorm.normal = (partPosition.rotation()*normal).block<3,1>(0,0);
                    int x,y,z;
                    toCoordinatePCLGrid(pointNorm.position(0),x);
                    toCoordinatePCLGrid(pointNorm.position(1),y);
                    toCoordinatePCLGrid(pointNorm.position(2),z);
                    if (config.verbose==1){
                        std::cout << "(" << pointNorm.position(0) << ", " << pointNorm.position(1) << ", " << pointNorm.position(2) << ") Update octree, xyz: " << x << ", " << y << ", " << z << "\n";
                    }
                    (*octreeGrid)(x,y,z).push_back(pointNorm);
                }
            }
        }
    }
}

/// filter voxel grid
void ObjectCompositionOctree::filterPCLGrid(void){
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
                            for (auto point : means){//compute angle between vectors and add select group
                                double angle = fabs(acos(point.normal.adjoint()*pointNorm.normal));
                                if (angle<config.maxAngleGrid)
                                    groupNo=iterNo;
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
                    (*octreeGrid)(idX,idY,idZ) = means;
                    for (auto& point : means){//update parts octree
                        int x,y,z;
                        toCoordinate(point.position(0),x,0); toCoordinate(point.position(1),y,0); toCoordinate(point.position(2),z,0);
                        (*octrees[0])(x,y,z).cloud.push_back(point);
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
    for (int idX=0; idX<(*octrees[layerNo]).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octrees[layerNo]).size(); idY++){
            for (int idZ=0; idZ<(*octrees[layerNo]).size(); idZ++){
                if ((*octrees[layerNo]).at(idX,idY,idZ).id>=0){
                    int wordId=0;
                    double minDist=std::numeric_limits<double>::max();
                    for (auto& word: vocabulary){
                        Mat34 estTransform;
                        /*ViewIndependentPart partA = word;
                        //transform
                        Vec3 trans(0.0,0.0,0.0);
                        double rot[3]={1.2,-0.7,0.6};
                        Eigen::Matrix3d m;
                        m = Eigen::AngleAxisd(rot[0], Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(rot[1], Eigen::Vector3d::UnitY())* Eigen::AngleAxisd(rot[2], Eigen::Vector3d::UnitZ());
                        int itP = 0;
                        for (auto& patch : partA.cloud){
                            patch.position = m*patch.position + trans;
                            patch.normal = m*patch.normal;
                            std::cout << "word point poisition " << word.cloud[itP].position.transpose() << "\n";
                            std::cout << "partA point poisition " << patch.position.transpose() << "\n";
                            itP++;
                        }
                        std::cout << "ref rot\n" << m << "\n";*/
                        double fitness = ViewIndependentPart::distanceGICP(word,(*octrees[layerNo])(idX,idY,idZ),config.configGICP,estTransform);
                        //std::cout << "estTransform\n" << estTransform.matrix() << "\n";
                        //std::cout << "fitnes " << fitness << "\n";
                        //getchar();
                        if (fitness<minDist){//find min distance
                            minDist=fitness;
                            //std::cout << "update " << idX << " " << idY << " " << idZ << " " << (*octrees[layerNo]).at(idX,idY,idZ).id << "\n";
                            (*octrees[layerNo])(idX,idY,idZ).id = wordId;
                            (*octrees[layerNo])(idX,idY,idZ).offset=estTransform;
                        }
                        wordId++;
                    }
                }
            }
        }
    }
}

/// get set of ids for the given input point
void ObjectCompositionOctree::getPartsIds(const Vec3& point, std::vector<int>& ids) const{
    int x,y,z;
    for (int layNo=0;layNo<3;layNo++){
        toCoordinate(point(0),x, layNo);    toCoordinate(point(1),y, layNo);    toCoordinate(point(2),z, layNo);
        //std::cout << "coordinate " << x << " " << y << " " << z << "\n";
        if (x%3==0) x+=1; else if (x%3==2) x-=1;
        if (y%3==0) y+=1; else if (y%3==2) y-=1;
        if (z%3==0) z+=1; else if (z%3==2) z-=1;
        ids.push_back((*octrees[layNo]).at(x,y,z).id);//4th layer
        /*if ((*octrees[layNo]).at(x,y,z).id>=0){
            std::cout << "lay no id" << layNo << " " << (*octrees[layNo]).at(x,y,z).id << "\n";
            getchar();
        }*/
    }
}

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinate(double pos, int& coord, int layerNo) const{
    coord = int(pos/(pow(3.0,layerNo)*config.voxelSize))+(int)(config.voxelsNo/(2*pow(3.0,layerNo)));
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
void ObjectCompositionOctree::fromCoordinatePCLGrid(int coord, double& pos) const{
    pos = (config.voxelSizeGrid)*(coord-config.voxelsNoGrid/(2))+(config.voxelSizeGrid/2.0);
}

/// update ids in the octree using new vocabulary
void ObjectCompositionOctree::updateIds(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy){
    for (int idX=0; idX<(*octrees[layerNo]).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octrees[layerNo]).size(); idY++){
            for (int idZ=0; idZ<(*octrees[layerNo]).size(); idZ++){
                if ((*octrees[layerNo]).at(idX,idY,idZ).id>=0){
                    //find part in the vocabulary
                   // int idOld = (*octrees[layerNo])(idX,idY,idZ).id;
                    (*octrees[layerNo])(idX,idY,idZ).id = findIdInVocabulary((*octrees[layerNo]).at(idX,idY,idZ), vocabulary);
                    if ((*octrees[layerNo])(idX,idY,idZ).layerId==5){//compute distance from centroid
                        Mat34 off;
                        ViewIndependentPart::distance(hierarchy.viewIndependentLayers[1][(*octrees[layerNo])(idX,idY,idZ).id], (*octrees[layerNo])(idX,idY,idZ), off);
                        (*octrees[layerNo])(idX,idY,idZ).offset = off;
                        //std::cout << "old id " << idOld << ", new id " << (*octrees[layerNo])(idX,idY,idZ).id << "\n";
                        //std::cout << "offset \n" << (*octrees[layerNo])(idX,idY,idZ).offset.matrix() << "\n";
                        //(*octrees[layerNo])(idX,idY,idZ).offset=Mat34::Identity();
                        //getchar();
                    }
                    if ((*octrees[layerNo])(idX,idY,idZ).layerId==6){//compute distance from centroid
                        Mat34 off;
                        ViewIndependentPart::distance(hierarchy.viewIndependentLayers[2][(*octrees[layerNo])(idX,idY,idZ).id], (*octrees[layerNo])(idX,idY,idZ), hierarchy.viewIndependentLayers[1], off);
                        (*octrees[layerNo])(idX,idY,idZ).offset = off;
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
    for (int idX=0; idX<(*octrees[layerNo]).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octrees[layerNo]).size(); idY++){
            for (int idZ=0; idZ<(*octrees[layerNo]).size(); idZ++){
                if ((*octrees[layerNo]).at(idX,idY,idZ).id>=0){
                    parts.push_back((*octrees[layerNo]).at(idX,idY,idZ));
                }
            }
        }
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
void ObjectCompositionOctree::getClusters(int layerNo, std::vector< std::set<int>>& clusters){
    for (int idX=0; idX<(*octrees[layerNo]).size(); idX++){///to do z-slicing
        for (int idY=0; idY<(*octrees[layerNo]).size(); idY++){
            for (int idZ=0; idZ<(*octrees[layerNo]).size(); idZ++){
                if ((*octrees[layerNo]).at(idX,idY,idZ).parts.size()>0){
                    std::set<int> partIds1, partIds2;
                    int iterNo=0;
                    Vec3 norm1;
                    for (auto & part : (*octrees[layerNo])(idX,idY,idZ).parts){
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
int ObjectCompositionOctree::createFirstLayerPart(ViewIndependentPart& newPart, int x, int y, int z){
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
                if ((*octrees[0]).at(x+i,y+j,z+k).cloud.size()>0){
                    for (auto& patch : (*octrees[0])(x+i,y+j,z+k).cloud){
                        //std::cout << "config.voxelSize " << config.voxelSize << "\n";
                        //std::cout << "config.voxelSizeGrid " << config.voxelSizeGrid << "\n";
                        //std::cout << config.voxelSize / config.voxelSizeGrid << "\n";
                        //std::cout << "coeff " << coeff << "\n";
                        //std::cout << "xyz " << x << " " << y << " " << z << "\n";
                        //std::cout << "grid coord " << gridCoord[0] << " " << gridCoord[1] << " " << gridCoord[2] << "\n";
                        //std::cout << "center oct " << center[0] << " " << center[1] << " " << center[2] << "\n";
                        //fromCoordinatePCLGrid(gridCoord[0],center[0]); fromCoordinatePCLGrid(gridCoord[1],center[1]); fromCoordinatePCLGrid(gridCoord[2],center[2]);
                        //std::cout << "center grid " << center[0] << " " << center[1] << " " << center[2] << "\n";
                        //std::cout << patch.position.transpose() << " bef\n";
                        for (int coord = 0; coord<3; coord++)
                            patch.position(coord)-=center[coord];
                        //std::cout << patch.position.transpose() << " aft\n";
                        //getchar();
                        newPart.cloud.push_back(patch);
                        partsNo++;
                    }
                    //newPart.cloud.insert(newPart.cloud.end(),newPatches.begin(), newPatches.end());
                    newPart.partIds[i+1][j+1][k+1] = 1;
                }
                else
                    newPart.partIds[i+1][j+1][k+1] = -1;
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
    for (int idX=1; idX<(*octrees[0]).size()-1; idX+=3){///to do z-slicing
        for (int idY=1; idY<(*octrees[0]).size()-1; idY+=3){
            for (int idZ=1; idZ<(*octrees[0]).size()-1; idZ+=3){
                ViewIndependentPart newPart;
                newPart.layerId=3;
                // add neighbouring parts into structure
                if (createFirstLayerPart(newPart, idX, idY, idZ)>0){
                    if (newPart.cloud.size()>(unsigned)config.minPatchesNo){
                        newPart.id = tempId;//hierarchy.interpreter.at((*octrees[destLayerNo-1]).at(idX, idY, idZ).id);
                        (*octrees[0])(idX, idY, idZ) = newPart;//update octree
                        vocabulary.push_back(newPart);
                        tempId++;
                    }
                }
            }
        }
    }
}

/// create next layer vocabulary
void ObjectCompositionOctree::createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary){
    vocabulary.clear();
    //for no reason take the middle part. In the feature it should be max, or weighted combination
    if (destLayerNo==0){
        std::cout << "create first layer vocabulary\n";
        filterPCLGrid();
        createFirstLayer(vocabulary);
        std::cout << "create first layer vocabulary finished\n";
    }
    else{
        int tempId=0;
        int iterx=0;
        // update next layer octree
        for (int idX=1; idX<(*octrees[destLayerNo-1]).size()-1; idX+=3){///to do z-slicing
            int itery=0;
            for (int idY=1; idY<(*octrees[destLayerNo-1]).size()-1; idY+=3){
                int iterz=0;
                for (int idZ=1; idZ<(*octrees[destLayerNo-1]).size()-1; idZ+=3){
                    ViewIndependentPart newPart;
                    newPart.layerId=destLayerNo+4;
                    // add neighbouring parts into structure
                    if (assignPartNeighbours(newPart, hierarchy, destLayerNo-1, idX, idY, idZ)>0){
                        newPart.id = tempId;//hierarchy.interpreter.at((*octrees[destLayerNo-1]).at(idX, idY, idZ).id);
                        if (newPart.cloud.size()>(unsigned)config.minPatchesNo){
                            (*octrees[destLayerNo])(iterx,itery,iterz) = newPart;//update octree
                            vocabulary.push_back(newPart);
                            tempId++;
                        }
                    }
                    iterz++;
                }
                itery++;
            }
            iterx++;
        }
    }
}

/// assign neighbouring parts to new part
int ObjectCompositionOctree::assignPartNeighbours(ViewIndependentPart& partVoxel, const Hierarchy& hierarchy, int layerNo, int x, int y, int z){
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
                if ((*octrees[layerNo]).at(x+i,y+j,z+k).id>=0){
                    //assign neighbouring ids
                    if (layerNo==0)
                        partVoxel.partIds[i+1][j+1][k+1] = hierarchy.interpreter.at((*octrees[layerNo]).at(x+i,y+j,z+k).id);
                    else
                        partVoxel.partIds[i+1][j+1][k+1] = (*octrees[layerNo]).at(x+i,y+j,z+k).id;
                    // assign spatial relation
                    if ((*octrees[layerNo]).at(x+i,y+j,z+k).id!=-1){
                        //if (i==0&&j==0&&k==0){
                        //    partVoxel.neighbourPoses[i+1][j+1][k+1] = (*octrees[layerNo]).at(x,y,z).pose;//set global pose
                        //}
                        //else {
                            partVoxel.neighbourPoses[i+1][j+1][k+1] = partPoseInv*(*octrees[layerNo]).at(x+i,y+j,z+k).pose;//set relative pose
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
        }
    }
    return partsNo;
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(void) {
    composition.reset(new ObjectCompositionOctree());
    return composition.get();
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(std::string config) {
    composition.reset(new ObjectCompositionOctree(config));
    return composition.get();
}
