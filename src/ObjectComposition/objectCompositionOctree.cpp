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
    octree.reset(new Octree<double>(config.voxelsNo));
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
    octree.reset(new Octree<double>(2));
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
}

/// update composition from octets (words from last view-independent layer's vocabulary)
void ObjectCompositionOctree::update(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, const Hierarchy& hierarchy){
    for (auto & part : parts){
        Mat34 partPosition(Mat34::Identity());
        //ViewDependentPart.gaussians[1][1].mean;
        Vec3 pos3d;
        camModel.getPoint(part.gaussians[1][1].mean, pos3d);
        Vec3 normal; hierarchy.getNormal(part, normal);
        Mat33 rot; normal2rot(normal, rot);
        Mat34 part3D;
        part3D.translation() = pos3d;
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                part3D(i,j) = rot(i,j);
            }
        }
        partPosition = cameraPose * part3D;//global position of the part
    }
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
