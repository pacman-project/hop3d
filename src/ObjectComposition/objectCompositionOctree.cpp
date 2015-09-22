#include "ObjectComposition/objectCompositionOctree.h"
#include <ctime>

using namespace hop3d;

/// A single instance of object composition
ObjectCompositionOctree::Ptr composition;

ObjectCompositionOctree::ObjectCompositionOctree(void) : ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE) {
}

/// Construction
ObjectCompositionOctree::ObjectCompositionOctree(std::string config) :
        ObjectComposition("Octree Object Composition", COMPOSITION_OCTREE), config(config) {
}

/// Destruction
ObjectCompositionOctree::~ObjectCompositionOctree(void) {
}

///config class constructor
ObjectCompositionOctree::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load part selector config file.\n";
    tinyxml2::XMLElement * group = config.FirstChildElement( "ObjectComposition" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load part selector parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryDoubleAttribute("voxelSize", &voxelSize);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("voxelsNo", &voxelsNo);
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(void) {
    composition.reset(new ObjectCompositionOctree());
    return composition.get();
}

hop3d::ObjectComposition* hop3d::createObjectCompositionOctree(std::string config) {
    composition.reset(new ObjectCompositionOctree(config));
    return composition.get();
}
