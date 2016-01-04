/** @file partSelectorMean.h
 *
 * implementation - part selector k-mean
 *
 */

#ifndef PART_SELECTOR_MEAN_H_INCLUDED
#define PART_SELECTOR_MEAN_H_INCLUDED

#include "hop3d/PartSelector/partSelector.h"
#include "tinyXML/tinyxml2.h"

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
    void selectParts(ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo);

    /// Select parts from the initial vocabulary
    void selectParts(ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy);

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
            /// Clustering -- max number of iterations
            int maxIter;
            /// Dictance measure
            int distanceMetric;
            /// Numbber of layers
            int layersNo;
            /// use compression rate
            std::vector<bool> useCompressionRate;
            /// clusters numbers
            std::vector<int> clustersNo;
            /// compressionRate
            std::vector<double> compressionRate;
    };

private:

private:
    ///Configuration of the module
    Config config;

    /// assign parts to clusters according to given cetroid
    void fit2clusters(const std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<ViewDependentPart::Seq>& clusters);

    /// assign parts to clusters according to given cetroid
    void fit2clusters(const std::vector<int>& centroids, const ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<ViewIndependentPart::Seq>& clusters);

    /// compute centroids for give clusters
    void computeCentroids(const std::vector<ViewDependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy);

    /// compute centroids for give clusters
    void computeCentroids(const std::vector<ViewIndependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    bool isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter);

    ///find new center of cluster
    int centerOfCluster(const std::set<int>& cluster, const ViewDependentPart::Seq& vocabulary, const Hierarchy& hierarchy) const;
};
}
#endif // PART_SELECTOR_MEAN_H_INCLUDED
