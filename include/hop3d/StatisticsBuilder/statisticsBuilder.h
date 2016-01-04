/** @file statisticsBuilder.h
 *
 * Statistics Builder interface
 *
 */

#ifndef _STATISTICS_BUILDER_H_
#define _STATISTICS_BUILDER_H_

#include "../Data/Defs.h"
#include "../Data/Vocabulary.h"

namespace hop3d {

/// Statistics builder interface
class StatsBuilder {
public:

    /// Statistics builder type
    enum Type {
        /// unbiased statistics builder
        STATS_UNBIASED,
    };

    /// overloaded constructor
    StatsBuilder(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the map
    virtual const std::string& getName() const {return name;}

    /// compute statistics for the set of octets
    virtual void computeStatistics(const std::vector<Octet>& octets, int layerNo, int startId, ViewDependentPart::Seq& dictionary) = 0;

    /// compute statistics for the set of octets
    virtual void computeStatistics(const std::vector<ViewIndependentPart>& elements, int layerNo, int startId, ViewIndependentPart::Seq& dictionary) = 0;

    /// Virtual descrutor
    virtual ~StatsBuilder() {
    }

protected:
    /// Map name
    const std::string name;

    /// Map type
    Type type;
};
};

#endif // _STATISTICS_BUILDER_H_
