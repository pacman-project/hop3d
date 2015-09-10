/** @file unbiasedStatsBuilder.h
 *
 * implementation - unbiased statistics builder
 *
 */

#ifndef UNBIASED_STATISTICS_BUILDER_H_INCLUDED
#define UNBIASED_STATISTICS_BUILDER_H_INCLUDED

#include "statisticsBuilder.h"
#include "../../external/tinyXML/tinyxml2.h"

namespace hop3d {
    /// create a single unbiased statisctics builder
    StatsBuilder* createUnbiasedStatsBuilder(void);
    /// create a single unbiased statisctics builder
    StatsBuilder* createUnbiasedStatsBuilder(std::string config);
}

using namespace hop3d;

/// Unbiased statistics implementation
class UnbiasedStatsBuilder: public StatsBuilder {
public:
    /// Pointer
    typedef std::unique_ptr<UnbiasedStatsBuilder> Ptr;

    /// Construction
    UnbiasedStatsBuilder(void);

    /// Construction
    UnbiasedStatsBuilder(std::string config);

    /// Destruction
    ~UnbiasedStatsBuilder(void);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
    };

private:
    ///Configuration of the module
    Config config;
};

#endif // UNBIASED_STATISTICS_BUILDER_H_INCLUDED
