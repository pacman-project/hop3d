/** @file partSelectorAgglomerative.h
 *
 * implementation - part selector agglomerative hierarchical clustering
 *
 */

#ifndef PART_SELECTOR_AGGLOMERATIVE_H_INCLUDED
#define PART_SELECTOR_AGGLOMERATIVE_H_INCLUDED

#include "hop3d/PartSelector/partSelector.h"
#include "tinyXML/tinyxml2.h"
#include <queue>

namespace hop3d {
    /// create a single part selector
    PartSelector* createPartSelectorAgglomerative(void);
    /// create a single part selector
    PartSelector* createPartSelectorAgglomerative(std::string config);


/// Unbiased statistics implementation
class PartSelectorAgglomerative: public PartSelector {
public:
    /// Pointer
    typedef std::unique_ptr<PartSelectorAgglomerative> Ptr;

    class DistanceElement{
    public:
        /// matched poses
        std::pair<int,int> partsIds;

        /// distance between poses, the smaller the higher priority in the queue
        double distance;

        bool operator() (const DistanceElement& elementA, const DistanceElement& elementB) const {
            if (elementA.distance>elementB.distance)
                return true;
            else
                return false;
        }
    };

    /// Construction
    PartSelectorAgglomerative(void);

    /// Construction
    PartSelectorAgglomerative(std::string config);

    /// Select parts from the initial vocabulary
    void selectParts(ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo);

    /// Select parts from the initial vocabulary
    void selectParts(ViewIndependentPart::Seq& dictionary, int layerNo);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy);

    /// Destruction
    ~PartSelectorAgglomerative(void);

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
            /// max distance
            std::vector<double> maxDist;
            /// max distance
            std::vector<double> maxClusterDist;
            /// config GICP
            ConfigGICP configGICP;
    };

private:

private:
    ///Configuration of the module
    Config config;
    /// distance priority queue
    std::priority_queue<DistanceElement, std::vector<DistanceElement>, DistanceElement > priorityQueueDistance;

    /// get clusters of parts id stored in octree (one cluster per voxel)
    bool isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter);

    ///find new center of cluster
    int centerOfCluster(const std::set<int>& cluster, const ViewDependentPart::Seq& vocabulary, const Hierarchy& hierarchy) const;

    /// compute distance matrix
    void computeDistanceMatrix(const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<std::vector<double>>& distanceMatrix, std::vector<std::vector<Mat34>>& transformMatrix);

    /// compute distance matrix for view-independent parts
    void computeDistanceMatrix(const ViewIndependentPart::Seq& dictionary, std::vector<std::vector<double>>& distanceMatrix, std::vector<std::vector<Mat34>>& transformMatrix);

    /// find min distance int the distance matrix
    double findMinDistance(const std::vector<std::vector<double>>& distanceMatrix, std::pair<int,int>& pairedIds);

    /// find clusters ids to which contain specyfic parts (pairedIds)
    void findPartsInClusters(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& pairedIds, std::pair<int,int>& clustersIds) const;

    /// merge two clusters
    bool mergeTwoClusters(std::vector<std::vector<int>>& clusters, const std::vector<int>& centroids, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix, int layerNo) const;

    /// compute centroids using pre-computed distance in distance matrix
    void computeCentroids(const std::vector<std::vector<int>>& clusters, const std::vector<std::vector<double>>& distanceMatrix, std::vector<int>& centroids) const;

    /// compute centroids using pre-computed distance in distance matrix
    void computeCentroid(const std::vector<int>& cluster, const std::vector<std::vector<double>>& distanceMatrix, int& centroid) const;

    /// update centroids
    void updateCentroids(const std::vector<std::vector<int>>& clusters, const std::vector<std::vector<double>>& distanceMatrix, std::vector<int>& centroids, const std::pair<int,int>& clustersIds) const;

    /// compute max distance between centroid of the first cluster and all parts in the second cluster
    double computeMaxDist(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<int>& centroids, const std::vector<std::vector<double>>& distanceMatrix) const;

    /// modify clusters according to min distance
    void reduceEntropy(std::vector<std::vector<int>>& clusters, const std::vector<int>& centroids, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix);
};
}
#endif // PART_SELECTOR_AGGLOMERATIVE_H_INCLUDED
