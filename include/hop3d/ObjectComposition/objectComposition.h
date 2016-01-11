/** @file objectComposition.h
 *
 * Object Composition interface
 *
 */

#ifndef _OBJECT_COMPOSITION_H_
#define _OBJECT_COMPOSITION_H_

#include "hop3d/Data/Defs.h"
#include "hop3d/Data/Vocabulary.h"
#include "hop3d/Data/Graph.h"
#include "hop3d/Utilities/depthSensorModel.h"
#include <set>

namespace hop3d {

/// Object Composition interface
class ObjectComposition {
public:

    /// Object Composition type
    enum Type {
        /// octree composition
        COMPOSITION_OCTREE,
    };

    /// overloaded constructor
    ObjectComposition(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the part composition
    virtual const std::string& getName() const {return name;}

    /// update composition from octets (words from last view-independent layer's vocabulary)
    virtual void update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, Hierarchy& hierarchy) = 0;

    /// update voxel grid which contains point and normals
    virtual void updatePCLGrid(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose) = 0;

    /// get clusters of parts id stored in octree (one cluster per voxel)
    virtual void getClusters(int layerNo, std::vector< std::set<int>>& clusters) = 0;

    /// create next layer vocabulary
    virtual void createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary) = 0;

    /// get octree in layer layerNo
    virtual void getParts(int layerNo, std::vector<ViewIndependentPart>& parts) = 0;

    /// update ids in the octree using new vocabulary
    virtual void updateIds(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy) = 0;

    /// get set of ids for the given input point
    virtual void getPartsIds(const Vec3& point, std::vector<int>& ids) const = 0;

    /// upodate voxel poses using new vocabulary
    virtual void updateVoxelsPose(int layerNo, const std::vector<ViewIndependentPart>& vocabulary) = 0;

    /// Virtual descrutor
    virtual ~ObjectComposition() {
    }

protected:
    /// Object Composition name
    const std::string name;

    /// Object Composition type
    Type type;
};
};

#endif // _OBJECT_COMPOSITION_H_
