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

    /// compute statistics for the set of octets
    void computeStatistics(const std::vector<Octet>& octets, ViewDependentPart::Seq& dictionary);

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
    /// Is octet in vector
    bool isOctetInGroups(const Octet& octet, std::vector<Octet::Seq>& groups, std::vector<Octet::Seq>::iterator& groupIt) const;

    /// Compute distance between octets
    double distance(const Octet& octetA, const Octet& octetB) const;

    /// Compute Gaussians for the grooup of octets
    void computeGaussians(Octet::Seq& group, ViewDependentPart& part) const;

    /// compute Gaussian parameters
    void computeGaussian(const Octet::Seq& group, Gaussian3D& gauss, unsigned int u, unsigned int v) const;

private:
    ///Configuration of the module
    Config config;
};

#endif // UNBIASED_STATISTICS_BUILDER_H_INCLUDED
