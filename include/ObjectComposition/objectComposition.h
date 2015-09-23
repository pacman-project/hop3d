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
    virtual const std::string& getName() const {return name;};

    /// update composition from octets (words from last view-independent layer's vocabulary)
    virtual void update(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose, const DepthSensorModel& camModel, const Hierarchy& hierarchy) = 0;

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
