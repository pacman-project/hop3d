/** @file partSelector.h
 *
 * Part Selector interface
 *
 */

#ifndef _PART_SELECTOR_H_
#define _PART_SELECTOR_H_

#include "Data/Defs.h"
#include "Data/Vocabulary.h"
#include "Data/Graph.h"
#include <set>

namespace hop3d {

/// Part selector interface
class PartSelector {
public:

    /// Part selector type
    enum Type {
        /// k-mean part selector
        SELECTOR_MEAN,
    };

    /// overloaded constructor
    PartSelector(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the part selector
    virtual const std::string& getName() const {return name;}

    /// Select parts from the initial vocabulary
    virtual void selectParts(ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo) = 0;

    /// Select parts from the initial vocabulary
    virtual void selectParts(ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo) = 0;

    /// get clusters of parts id stored in octree (one cluster per voxel)
    virtual void createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy) = 0;

    /// Virtual descrutor
    virtual ~PartSelector() {
    }

protected:
    /// Map name
    const std::string name;

    /// Map type
    Type type;
};
};

#endif // _PART_SELECTOR_H_
