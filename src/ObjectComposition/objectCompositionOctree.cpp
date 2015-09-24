#include "ObjectComposition/objectCompositionOctree.h"
#include <ctime>

using namespace hop3d;

/// A single instance of object composition
ObjectCompositionOctree::Ptr composition;

ObjectCompositionOctree::ObjectCompositionOctree(void) : ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE) {
}

/// Construction
ObjectCompositionOctree::ObjectCompositionOctree(std::string _config) :
        ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE), config(_config) {
    octree.reset(new Octree<PartVoxel>(config.voxelsNo));
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
    octree.reset(new Octree<PartVoxel>(2));
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
void ObjectCompositionOctree::update(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, const Hierarchy& hierarchy){
    for (auto & part : parts){
        Mat34 partPosition(Mat34::Identity());
        Vec3 pos3d;
        std::cout << "part.id: " << part.id << "\n";
        camModel.getPoint(part.gaussians[1][1].mean, pos3d);
        Vec3 normal; hierarchy.getNormal(part, normal);
        Mat33 rot; normal2rot(normal, rot);
        Mat34 part3D;
        part3D.translation() = pos3d;//set position of the part
        for (int i=0;i<3;i++)//and orientation
            for (int j=0;j<3;j++)
                part3D(i,j) = rot(i,j);
        partPosition = cameraPose * part3D;//global position of the part
        int x = int(partPosition(0,3)/config.voxelSize);
        int y = int(partPosition(1,3)/config.voxelSize);
        int z = int(partPosition(2,3)/config.voxelSize);
        (*octree)(x,y,z).poses.push_back(partPosition);
        (*octree)(x,y,z).partIds.push_back(part.id);
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
void ObjectCompositionOctree::getClusters(std::vector< std::set<int>>& clusters){
    for (int idX=0; idX<config.voxelsNo; idX++){///to do z-slicing
        for (int idY=0; idY<config.voxelsNo; idY++){
            for (int idZ=0; idZ<config.voxelsNo; idZ++){;
                if ((*octree).at(idX,idY,idZ).partIds.size()>0){
                    std::set<int> partIds1, partIds2;
                    int iterNo=0;
                    Vec3 norm1;
                    for (auto & ids : (*octree)(idX,idY,idZ).partIds){
                        if (iterNo>0){
                            Vec3 norm2 = (*octree).at(idX,idY,idZ).poses[iterNo].matrix().block(0,2,3,1);
                            //compute angle between normals
                            double dp = norm1.adjoint()*norm2;
                            if (fabs(acos(dp))>config.maxAngle)// if angle between normals is bigger than threshold add to another cluster
                                partIds2.insert(ids);
                            else
                                partIds1.insert(ids);
                        }
                        else {
                            partIds1.insert(ids);
                            norm1=(*octree).at(idX,idY,idZ).poses[iterNo].matrix().block(0,2,3,1);
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

/// get clusters of parts id stored in octree (one cluster per voxel)
void ObjectCompositionOctree::createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary){
    vocabulary.clear();
    std::vector< std::set<int>> newClusters;
    for (auto & cluster : clusters){
        bool isInClusters(false);
        std::vector< std::set<int>>::iterator iter;
        for (auto & partId : cluster){
            //check if part is in vocabulary
            if (isInOctets(newClusters, partId, iter)){
                isInClusters = true;
                break;
            }
        }
        if (isInClusters){//if part is in vocabulary, update existing cluster
            for (auto & partId : cluster)
                (*iter).insert(partId);
        }
        else //else create new cluster
            newClusters.push_back(cluster);
    }
    //update vocabulary
    int idNo=0;
    for (auto & cluster : newClusters){
        ViewIndependentPart part;
        part.layerId=4;
        part.id = idNo;
        for (auto & partId : cluster){
            part.group.push_back(partId);
        }
        idNo++;
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
bool ObjectCompositionOctree::isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter){
    for (std::vector< std::set<int>>::iterator cluster = clusters.begin(); cluster!=clusters.end(); cluster++){
        for (std::set<int>::iterator partId = (*cluster).begin(); partId!=(*cluster).end();cluster++){
            if (*partId==id){
                iter = cluster;
                return true;
            }
        }
    }
    return false;
}

/// compute rotation matrix from normal vector ('y' axis is vetical)
void ObjectCompositionOctree::normal2rot(const Vec3& normal, Mat33& rot){
    Vec3 y(0,1,0); Vec3 x;
    Vec3 _normal(normal);
    x = _normal.cross(y);
    DepthSensorModel::normalizeVector(_normal);
    y = y.cross(_normal);
    DepthSensorModel::normalizeVector(x);
    rot.block(0,0,3,1) = x;
    rot.block(0,1,3,1) = y;
    rot.block(0,2,3,1) = _normal;
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(void) {
    composition.reset(new ObjectCompositionOctree());
    return composition.get();
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(std::string config) {
    composition.reset(new ObjectCompositionOctree(config));
    return composition.get();
}
