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
    typedef std::shared_ptr< Octree<ViewIndependentPart> > OctreePtr;

    /// Construction
    ObjectCompositionOctree(void);

    /// Construction
    ObjectCompositionOctree(std::string config);

    /// update composition from octets (words from last view-independent layer's vocabulary)
    void update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, Hierarchy& hierarchy);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void getClusters(int layerNo, std::vector< std::set<int>>& clusters);

    /// create next layer vocabulary
    void createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary);

    /// get octree in layer layerNo
    void getParts(int layerNo, std::vector<ViewIndependentPart>& parts) const;

    /// update ids in the octree using new vocabulary
    void updateIds(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy);

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
    //void normal2rot(const Vec3& normal, Mat33& rot);

    /// assign neighbouring parts to new part
    int assignPartNeighbours(ViewIndependentPart& partVoxel, const Hierarchy& hierarchy, int layerNo, int x, int y, int z);

    /// find part in the vocabulary and return new id
    int findIdInVocabulary(const ViewIndependentPart& part, const std::vector<ViewIndependentPart>& vocabulary);

    /// convert global coordinates to octree coordinates
    void toCoordinate(double pos, int& coord, int layerNo);

    /// convert octree coordinates to global coordinates
    void fromCoordinate(int coord, double& pos, int layero);
};
}
#endif // OBJECT_COMPOSITION_OCTREE_H_INCLUDED
