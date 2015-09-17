/** @file partSelectorMean.h
 *
 * implementation - part selector k-mean
 *
 */

#ifndef PART_SELECTOR_MEAN_H_INCLUDED
#define PART_SELECTOR_MEAN_H_INCLUDED

#include "PartSelector/partSelector.h"
#include "../../external/tinyXML/tinyxml2.h"

namespace hop3d {
    /// create a single part selector
    PartSelector* createPartSelectorMean(void);
    /// create a single part selector
    PartSelector* createPartSelectorMean(std::string config);


/// Unbiased statistics implementation
class PartSelectorMean: public PartSelector {
public:
    /// Pointer
    typedef std::unique_ptr<PartSelectorMean> Ptr;

    /// Construction
    PartSelectorMean(void);

    /// Construction
    PartSelectorMean(std::string config);

    /// Select parts from the initial vocabulary
    void selectParts(ViewDependentPart::Seq& dictionary, Hierarchy& hierarchy);

    /// Destruction
    ~PartSelectorMean(void);

    class Config{
      public:
        Config() : verbose(0){
        }

        Config(std::string configFilename);
        public:
            /// Verbose
            int verbose;
            /// Clusters no
            int clustersNo;
            /// Clustering -- max number of iterations
            int maxIter;
    };

private:

private:
    ///Configuration of the module
    Config config;

    /// assign parts to clusters according to given cetroid
    void fit2clusters(const std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<ViewDependentPart::Seq>& clusters);

    /// compute centroids for give clusters
    void computeCentroids(const std::vector<ViewDependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy);
};
}
#endif // PART_SELECTOR_MEAN_H_INCLUDED
