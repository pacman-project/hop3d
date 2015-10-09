/** @file objectComposition.h
 *
 * Object Composition interface
 *
 */

#ifndef _OBJECT_COMPOSITION_H_
#define _OBJECT_COMPOSITION_H_

#include "Data/Defs.h"
#include "Data/Vocabulary.h"
#include "Data/Graph.h"
#include "Utilities/depthSensorModel.h"
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

    /// get clusters of parts id stored in octree (one cluster per voxel)
    virtual void getClusters(int layerNo, std::vector< std::set<int>>& clusters) = 0;

    /// create next layer vocabulary
    virtual void createNextLayerVocabulary(int destLayerNo, const Hierarchy& hierarchy, std::vector<ViewIndependentPart>& vocabulary) = 0;

    /// get octree in layer layerNo
    virtual void getParts(int layerNo, std::vector<ViewIndependentPart>& parts) const = 0;

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
