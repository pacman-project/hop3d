#include "PartSelector/partSelectorMean.h"
#include <ctime>

using namespace hop3d;

/// A single instance of part selector
PartSelectorMean::Ptr selector;

PartSelectorMean::PartSelectorMean(void) : PartSelector("k-mean Part Selector", SELECTOR_MEAN) {
}

/// Construction
PartSelectorMean::PartSelectorMean(std::string config) :
        PartSelector("k-mean Part Selector", SELECTOR_MEAN), config(config) {
}

/// Destruction
PartSelectorMean::~PartSelectorMean(void) {
}

///config class constructor
PartSelectorMean::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load part selector config file.\n";
    tinyxml2::XMLElement * group = config.FirstChildElement( "PartSelector" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load part selector parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryIntAttribute("clustersNo", &clustersNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("maxIter", &maxIter);
}

/// Select parts from the initial vocabulary
void PartSelectorMean::selectParts(ViewDependentPart::Seq& dictionary, Hierarchy& hierarchy){
    if (config.verbose==1){
        std::cout << "Initial number of words in dictionary: " << dictionary.size() << "\n";
        std::cout << "Desired number of words: " << config.clustersNo << "\n";
    }
    std::vector<int> centroids(config.clustersNo);//id of part assigned to the centroid;
    std::srand ( unsigned ( std::time(0) ) );
    std::vector<int> partsIds(dictionary.size(), 0);
    for (size_t i=0;i<dictionary.size();i++)
        partsIds[i]=(int)i;
    std::random_shuffle( partsIds.begin(), partsIds.end() );
    for (size_t i=0;i<centroids.size();i++){//assign random centers of centroid
        centroids[i] = partsIds[i];
    }
    std::vector<ViewDependentPart::Seq> clusters(config.clustersNo);
    for (int i=0;i<config.maxIter;i++){
        if (config.verbose==2){
            std::cout << "centroids: ";
            for (size_t i=0;i<centroids.size();i++){
                std::cout << centroids[i] << ", ";
            }
            std::cout << "\n";
        }
        fit2clusters(centroids, dictionary, hierarchy, clusters);
        computeCentroids(clusters, centroids, hierarchy);
        if (config.verbose==2){
            std::cout << "centroids new: ";
            for (size_t i=0;i<centroids.size();i++){
                std::cout << centroids[i] << ", ";
            }
            std::cout << "\n";
        }
    }
    ViewDependentPart::Seq newDictionary;
    dictionary.clear();
    int centroidNo=0;
    for (auto it = clusters.begin(); it!=clusters.end();it++){
        ViewDependentPart part = dictionary[centroids[centroidNo]];
        for (auto itPart = it->begin(); itPart!=it->end();itPart++)
            part.group.push_back(*itPart);
        newDictionary.push_back(part);
        centroidNo++;
    }
    dictionary = newDictionary;
}

/// assign parts to clusters according to given cetroid
void PartSelectorMean::fit2clusters(const std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<ViewDependentPart::Seq>& clusters){
    for (size_t i=0;i<clusters.size();i++)
        clusters[i].clear();
    for (auto it = dictionary.begin();it!=dictionary.end();it++){// for each part
        double minDist= std::numeric_limits<double>::max();
        int clusterNo = 0;
        int centroidId = 0;
        for (auto itCentr = centroids.begin();itCentr!=centroids.end();itCentr++){//for each cluster
            if (it->layerId==2){//compute distance from centroid
                double dist = ViewDependentPart::distance(*it,dictionary[*itCentr],hierarchy.firstLayer);
                if (dist<minDist){
                    minDist = dist;
                    centroidId = clusterNo;
                }
            }
            clusterNo++;
        }
        clusters[centroidId].push_back(*it);
    }
    if (config.verbose==2){
        std::cout << "Elements no in clusters: ";
        for (size_t i=0;i<clusters.size();i++){
            std::cout << clusters[i].size() << ", ";
        }
        std::cout << "\n";
    }
}

/// compute centroids for give clusters
void PartSelectorMean::computeCentroids(const std::vector<ViewDependentPart::Seq>& clusters, std::vector<int>& centroids, const Hierarchy& hierarchy){
    int clusterNo=0;
    for (auto itClust = clusters.begin(); itClust!=clusters.end();itClust++){
        double distMin = std::numeric_limits<double>::max();
        int centerId=0;
        for (auto itPart = itClust->begin(); itPart!=itClust->end();itPart++){//compute mean dist for each part as a centroid
            double distSum = 0;
            for (auto itPart2 = itClust->begin(); itPart2!=itClust->end();itPart2++){//compute mean dist for each part as a centroid
                if (itPart->layerId==1){
                    distSum+=ViewDependentPart::distance(*itPart,*itPart2,hierarchy.firstLayer);
                }
            }
            if (distSum<distMin){
                distMin=distSum;
                centerId=itPart->id;
            }
        }
        clusterNo++;
        centroids[clusterNo]=centerId;
    }
}

hop3d::PartSelector* hop3d::createPartSelectorMean(void) {
    selector.reset(new PartSelectorMean());
    return selector.get();
}

hop3d::PartSelector* hop3d::createPartSelectorMean(std::string config) {
    selector.reset(new PartSelectorMean(config));
    return selector.get();
}
