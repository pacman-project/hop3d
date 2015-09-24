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

class PartVoxel{
public:
    std::vector<Mat34> poses;
    std::vector<int> partIds;

    PartVoxel(){}
    PartVoxel(int size){size=size-1;};// required by octree
};

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

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void getClusters(std::vector< std::set<int>>& clusters);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary);

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
            /// max angle between two parts (normals), if current angle is bigger than max second cluster is created
            double maxAngle;
    };

private:
    ///Configuration of the module
    Config config;
    /// Octree
    std::unique_ptr< Octree<PartVoxel> > octree;

    /// compute rotation matrix from normal vector ('y' axis is vetical)
    void normal2rot(const Vec3& normal, Mat33& rot);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    bool isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter);
};
}
#endif // OBJECT_COMPOSITION_OCTREE_H_INCLUDED
