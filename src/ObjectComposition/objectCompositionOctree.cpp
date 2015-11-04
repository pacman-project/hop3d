#include "ObjectComposition/objectCompositionOctree.h"
#include <ctime>
#include "ImageFilter/normalImageFilter.h"

using namespace hop3d;

/// A single instance of object composition
ObjectCompositionOctree::Ptr composition;

ObjectCompositionOctree::ObjectCompositionOctree(void) : ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE) {
}

/// Construction
ObjectCompositionOctree::ObjectCompositionOctree(std::string _config) :
        ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE), config(_config) {
    octrees.resize(3);
    octrees[0].reset(new Octree<ViewIndependentPart>(config.voxelsNo)); // cellSize0
    octrees[1].reset(new Octree<ViewIndependentPart>(config.voxelsNo/2)); // cellSize1=cellSize0*3;
    octrees[2].reset(new Octree<ViewIndependentPart>(config.voxelsNo/4)); // cellSize2=cellSize1*3=cellSize0*9;
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
    octrees.resize(3);
    octrees[0].reset(new Octree<ViewIndependentPart>(8)); // cellSize0
    octrees[1].reset(new Octree<ViewIndependentPart>(4)); // cellSize1=cellSize0*3;
    octrees[2].reset(new Octree<ViewIndependentPart>(2)); // cellSize2=cellSize1*3=cellSize0*9;
}

///config class constructor
ObjectCompositionOctree::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load Object Composition octree config file.\n";
    tinyxml2::XMLElement * group = config.FirstChildElement( "ObjectComposition" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load Object Composition octree parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryDoubleAttribute("voxelSize", &voxelSize);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("voxelsNo", &voxelsNo);
    group->FirstChildElement( "parameters" )->QueryDoubleAttribute("maxAngle", &maxAngle);
}

/// update composition from octets (words from last view-independent layer's vocabulary)
void ObjectCompositionOctree::update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, Hierarchy& hierarchy){
    for (auto & part : parts){
        Mat34 partPosition(Mat34::Identity());
        Vec3 pos3d;
        if (config.verbose==1){
            std::cout << "update octree, part.id: " << part.id << "\n";
        }
        Mat34 part3D(Mat34::Identity());
        Vec3 normal; hierarchy.getNormal(part, normal);
        part3D = NormalImageFilter::coordinateFromNormal(normal);
        Vec3 position(part.location.u, part.location.v, part.location.depth);
        camModel.getPoint(position, pos3d);
        part3D.translation() = pos3d;//set position of the part

        partPosition = cameraPose * part3D;//global position of the part
        int x,y,z;
        toCoordinate(partPosition(0,3),x, config.voxelSize);
        toCoordinate(partPosition(1,3),y, config.voxelSize);
        toCoordinate(partPosition(2,3),z, config.voxelSize);
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

/// convert global coordinates to octree coordinates
void ObjectCompositionOctree::toCoordinate(double pos, int& coord, double scale){
    coord = int(pos/scale)+config.voxelsNo/2;
}

/// convert octree coordinates to global coordinates
void ObjectCompositionOctree::fromCoordinate(int coord, double& pos, double scale){
    pos = scale*(coord-config.voxelsNo/2)+(scale/2.0);
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
                        ViewIndependentPart::distance(hierarchy.viewIndependentLayers[1][(*octrees[layerNo])(idX,idY,idZ).id], (*octrees[layerNo])(idX,idY,idZ), off, 0);
                        (*octrees[layerNo])(idX,idY,idZ).offset = off;
                        //std::cout << "old id " << idOld << ", new id " << (*octrees[layerNo])(idX,idY,idZ).id << "\n";
                        //std::cout << "offset \n" << (*octrees[layerNo])(idX,idY,idZ).offset.matrix() << "\n";
                        //(*octrees[layerNo])(idX,idY,idZ).offset=Mat34::Identity();
                        //getchar();
                    }
                    if ((*octrees[layerNo])(idX,idY,idZ).layerId==6){//compute distance from centroid
                        Mat34 off;
                        ViewIndependentPart::distance(hierarchy.viewIndependentLayers[2][(*octrees[layerNo])(idX,idY,idZ).id], (*octrees[layerNo])(idX,idY,idZ), vocabulary, off, 0);
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
void ObjectCompositionOctree::getParts(int layerNo, std::vector<ViewIndependentPart>& parts) const{
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
                            if (fabs(acos(dp))>config.maxAngle)// if angle between normals is bigger than threshold add to another cluster
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

/// create next layer vocabulary
void ObjectCompositionOctree::createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary){
    vocabulary.clear();
    //for no reason take the middle part. In the feature it should be max, or weighted combination
    int iterx=0;
    int tempId=0;
    // update next layer octree
    for (int idX=1; idX<(*octrees[destLayerNo-1]).size()-1; idX+=3){///to do z-slicing
        int itery=0;
        for (int idY=1; idY<(*octrees[destLayerNo-1]).size()-1; idY+=3){
            int iterz=0;
            for (int idZ=1; idZ<(*octrees[destLayerNo-1]).size()-1; idZ+=3){
                ViewIndependentPart newPart;
                newPart.layerId=destLayerNo+4;
                // add neighbouring parts into structure
                if (assignPartNeighbours(newPart, hierarchy, destLayerNo-1, idX, idY, idZ, pow(3.0,destLayerNo-1)*config.voxelSize)>0){
                    newPart.id = tempId;//hierarchy.interpreter.at((*octrees[destLayerNo-1]).at(idX, idY, idZ).id);
                    (*octrees[destLayerNo])(iterx,itery,iterz) = newPart;//update octree
                    vocabulary.push_back(newPart);
                    tempId++;
                }
                iterz++;
            }
            itery++;
        }
        iterx++;
    }
}

/// assign neighbouring parts to new part
int ObjectCompositionOctree::assignPartNeighbours(ViewIndependentPart& partVoxel, const Hierarchy& hierarchy, int layerNo, int x, int y, int z, double scale){
    Mat34 pose(Mat34::Identity());
    fromCoordinate(x, pose(0,3), scale);
    fromCoordinate(y, pose(1,3), scale);
    fromCoordinate(z, pose(2,3), scale);
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
