/** @file objectCompositionOctree.h
 *
 * implementation - object Composition Octree
 *
 */

#ifndef OBJECT_COMPOSITION_OCTREE_H_INCLUDED
#define OBJECT_COMPOSITION_OCTREE_H_INCLUDED

#include "ObjectComposition/objectComposition.h"
#include "../../external/tinyXML/tinyxml2.h"
#include "octree.h"

namespace hop3d {
    /// create a single part selector
    ObjectComposition* createObjectCompositionOctree(void);
    /// create a single part selector
    ObjectComposition* createObjectCompositionOctree(std::string config);

/// Object Composition implementation
class ObjectCompositionOctree: public ObjectComposition {
public:
    /// Pointer
    typedef std::unique_ptr<ObjectCompositionOctree> Ptr;

    /// Construction
    ObjectCompositionOctree(void);

    /// Construction
    ObjectCompositionOctree(std::string config);

    /// update composition from octets (words from last view-independent layer's vocabulary)
    void update(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, const Hierarchy& hierarchy);

    /// Destruction
    ~ObjectCompositionOctree(void);

    class Config{
      public:
        Config() : verbose(0){
        }

        Config(std::string configFilename);
        public:
            /// Verbose
            int verbose;
            /// voxel size
            double voxelSize;
            /// Clusters no -- second layer
            int voxelsNo;
    };

private:
    ///Configuration of the module
    Config config;
    /// Octree
    std::unique_ptr< Octree<double>> octree;

    /// compute rotation matrix from normal vector ('y' axis is vetical)
    void normal2rot(const Vec3& normal, Mat33& rot);
};
}
#endif // OBJECT_COMPOSITION_OCTREE_H_INCLUDED
