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
    class Part3D{
    public:
        /// pose of the part
        Mat34 pose;
        /// id of the part
        int id;

        Part3D(void){};
        Part3D(Mat34& _pose, int _id) : pose(_pose), id(_id){};
    };

    /// part composition
    std::vector<Part3D> parts;

    /// Part id
    int id;
    /// Layer id
    int layerId;

    /// id of neighbouring parts
    std::array<std::array<std::array<int,3>,3>,3> partIds;

    /// Gaussians related to positions of neighbouring parts
    std::array<std::array<std::array<GaussianSE3,3>,3>,3> gaussians;

    PartVoxel(){};
    PartVoxel(int size){size=size-1;};// required by octree
};

/// Object Composition implementation
class ObjectCompositionOctree: public ObjectComposition {
public:
    /// Pointer
    typedef std::unique_ptr<ObjectCompositionOctree> Ptr;
    typedef std::unique_ptr< Octree<PartVoxel> > OctreePtr;

    /// Construction
    ObjectCompositionOctree(void);

    /// Construction
    ObjectCompositionOctree(std::string config);

    /// update composition from octets (words from last view-independent layer's vocabulary)
    void update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, const Hierarchy& hierarchy);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void getClusters(int layerNo, std::vector< std::set<int>>& clusters);

    /// create next layer vocabulary
    void createNextLayerVocabulary(int destLayerNo, std::vector<ViewIndependentPart>& vocabulary);

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
    std::vector<OctreePtr> octrees;

    /// compute rotation matrix from normal vector ('y' axis is vetical)
    void normal2rot(const Vec3& normal, Mat33& rot);

};
}
#endif // OBJECT_COMPOSITION_OCTREE_H_INCLUDED
