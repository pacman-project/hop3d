/** @file unbiasedStatsBuilder.h
 *
 * implementation - unbiased statistics builder
 *
 */

#ifndef UNBIASED_STATISTICS_BUILDER_H_INCLUDED
#define UNBIASED_STATISTICS_BUILDER_H_INCLUDED

#include "statisticsBuilder.h"
#include "tinyXML/tinyxml2.h"

namespace hop3d {
    /// create a single unbiased statisctics builder
    StatsBuilder* createUnbiasedStatsBuilder(void);
    /// create a single unbiased statisctics builder
    StatsBuilder* createUnbiasedStatsBuilder(std::string config);


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
    void computeStatistics(const std::vector<Octet>& octets, int layerNo, int startId, ViewDependentPart::Seq& dictionary);

    /// compute statistics for the set of octets
    void computeStatistics(const std::vector<ViewIndependentPart>& elements, int layerNo, int startId, ViewIndependentPart::Seq& dictionary);

    /// compute statistics for the set of octets
    void vocabularyFromOctets(const std::vector<Octet>& octets, int layerNo, int startId, ViewDependentPart::Seq& dictionary);

    /// Destruction
    ~UnbiasedStatsBuilder(void);

    class Config{
      public:
        Config() : verbose(0){
        }

        Config(std::string configFilename);
        public:
            /// Verbose
            int verbose;
            /// use Euclidean coordinates to define part position
            bool useEuclideanCoordinates;
    };

private:
    /// Is octet in vector
    //bool isOctetInGroups(const Octet& octet, std::vector<Octet::Seq>& groups, std::vector<Octet::Seq>::iterator& groupIt) const;
    /// is word in vector
    template<class T>
    bool isElementInGroups(const T& element, std::vector<typename T::Seq>& groups, typename std::vector<std::vector<T>>::iterator& groupIt) const{
        for (typename std::vector<std::vector<T>>::iterator it = groups.begin(); it!=groups.end(); it++){
            if (element.partIds==it->back().partIds){
                groupIt = it;
                return true;
            }
        }
        return false;
    }

    /// compute statistics for the set of elements
    template<class T>
    void groupElements(const std::vector<T>& elements, std::vector<typename T::Seq>& groups){
        typename std::vector<typename T::Seq>::iterator groupIter;
        if (config.verbose==1){
            std::cout << "Start ocetes grouping (n=" << elements.size() << ")...\n";
        }
        int iterNo=1;
        for (auto it=elements.begin();it!=elements.end();it++){ // grouping
            if(!isElementInGroups(*it, groups, groupIter)){
                typename T::Seq group; group.push_back(*it);
                groups.push_back(group);
            }
            else
                (*groupIter).push_back(*it);
            if (config.verbose==1){
                if ((elements.size()>10)&&iterNo%int(elements.size()/10)==0){
                    std::cout << "Iteration: " << iterNo << "/" << elements.size() << "\n";
                }
            }
            iterNo++;
        }
        if (config.verbose==1){
            std::cout << "done.\n";
        }
    }

    /// Compute Gaussians for the grooup of octets
    void computeGaussians(Octet::Seq& group, ViewDependentPart& part) const;

    /// Compute Gaussians for the group of view independent parts
    void computeGaussians(ViewIndependentPart::Seq& group, ViewIndependentPart& part) const;

    /// compute Gaussian parameters
    void computeGaussian(const Octet::Seq& group, Gaussian3D& gauss, unsigned int u, unsigned int v) const;

    /// compute Gaussian parameters
    void computeGaussian(const ViewIndependentPart::Seq& group, GaussianSE3& gauss, unsigned int x, unsigned int y, unsigned int z) const;

    /// compute mean SE3 transformations from gaussians
    void computeNeigbourPoses(ViewIndependentPart& part) const;

private:
    /// Configuration of the module
    Config config;
};
}
#endif // UNBIASED_STATISTICS_BUILDER_H_INCLUDED
