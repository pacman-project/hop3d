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
    group->FirstChildElement( "parameters" )->QueryIntAttribute("clustersSecondLayer", &clustersSecondLayer);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("clustersThirdLayer", &clustersThirdLayer);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("maxIter", &maxIter);
}

/// Select parts from the initial vocabulary
void PartSelectorMean::selectParts(ViewDependentPart::Seq& dictionary, Hierarchy& hierarchy, int layerNo){
    int clustersNo = (layerNo==2) ? config.clustersSecondLayer : config.clustersThirdLayer;
    if (config.verbose>0){
        std::cout << "Initial number of words in dictionary: " << dictionary.size() << "\n";
        std::cout << "Desired number of words: " << clustersNo << "\n";
    }
    std::vector<int> centroids(clustersNo);//index in dictionary of part assigned to the centroid;
    std::srand ( unsigned ( std::time(0) ) );
    std::vector<int> partsIds(dictionary.size(), 0);
    for (size_t i=0;i<dictionary.size();i++)
        partsIds[i]=(int)i;
    std::random_shuffle( partsIds.begin(), partsIds.end() );
    for (size_t i=0;i<centroids.size();i++){//assign random centers of centroid
        centroids[i] = partsIds[i];
    }
    std::vector<ViewDependentPart::Seq> clusters(clustersNo);
    for (int i=0;i<config.maxIter;i++){
        if (config.verbose==1){
            if ((config.maxIter>10)&&i%((config.maxIter+1)/10)==0){
                std::cout << "Iteration: " << i+1 << "/" << config.maxIter << "\n";
            }
        }
        if (config.verbose==2){
            std::cout << "centroids: ";
            for (size_t j=0;j<centroids.size();j++){
                std::cout << dictionary[centroids[j]].id << ", ";
            }
            std::cout << "\n";
        }
        fit2clusters(centroids, dictionary, hierarchy, clusters);
        computeCentroids(clusters, centroids, dictionary, hierarchy);
        if (config.verbose==2){
            std::cout << "centroids new: ";
            for (size_t j=0;j<centroids.size();j++){
                std::cout << dictionary[centroids[j]].id << ", ";
            }
            std::cout << "\n";
        }
    }
    ViewDependentPart::Seq newDictionary;
    dictionary.clear();
    int centroidNo=0;
    for (auto it = clusters.begin(); it!=clusters.end();it++){
        if (it->size()>0){
            ViewDependentPart part = dictionary[centroids[centroidNo]];
            for (auto itPart = it->begin(); itPart!=it->end();itPart++){
                part.group.push_back(*itPart);
            }
            newDictionary.push_back(part);
            centroidNo++;
        }
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
            double dist = 0;
            if (it->layerId==2){//compute distance from centroid
                dist = ViewDependentPart::distance(*it,dictionary[*itCentr],hierarchy.firstLayer);
            }
            else if (it->layerId==3){//compute distance from centroid
                dist = ViewDependentPart::distance(*it,dictionary[*itCentr], dictionary, hierarchy.firstLayer);
            }
            if (dist<minDist){
                minDist = dist;
                centroidId = clusterNo;
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
        std::cout << "Clusters: ";
        for (size_t i=0;i<clusters.size();i++){
            std::cout << "{" << i << ": ";
            for (size_t j=0;j<clusters[i].size();j++){
                std::cout << clusters[i][j].id;
                std::cout << ", ";
            }
            std::cout << "},";
        }
        std::cout << "\n";
    }
}

/// compute centroids for give clusters
void PartSelectorMean::computeCentroids(const std::vector<ViewDependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy){
    int clusterNo=0;
    for (auto itClust = clusters.begin(); itClust!=clusters.end();itClust++){ //for each cluster
        double distMin = std::numeric_limits<double>::max();
        int centerId=0;
        for (auto itPart = itClust->begin(); itPart!=itClust->end();itPart++){//for each part in cluster
            double distSum = 0; //compute new centroid
            for (auto itPart2 = itClust->begin(); itPart2!=itClust->end();itPart2++){//compute mean dist for each part as a centroid
                if (itPart->layerId==2){
                    distSum+=ViewDependentPart::distance(*itPart,*itPart2,hierarchy.firstLayer);
                }
                else if (itPart->layerId==3){
                    distSum+=ViewDependentPart::distance(*itPart,*itPart2, dictionary, hierarchy.firstLayer);
                }
            }
            if (distSum<distMin){
                distMin=distSum;
                centerId=itPart->id;
            }
        }
        //find part in vocabulary
        for (size_t i=0;i<dictionary.size();i++){
            if (dictionary[i].id==centerId){
                centroids[clusterNo]=(int)i;
                break;
            }
        }
        clusterNo++;
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
void PartSelectorMean::createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy){
    vocabulary.clear();
    std::vector< std::set<int> > newClusters;
    for (auto & cluster : clusters){
        bool isInClusters(false);
        std::vector< std::set<int> >::iterator iter;
        for (auto & partId : cluster){
            //check if part is in vocabulary
            if (isInOctets(newClusters, partId, iter)){
                isInClusters = true;
                break;
            }
        }
        if (isInClusters){//if part is in vocabulary, update existing cluster
            for (auto & partId : cluster){
                (*iter).insert(partId);
            }
        }
        else //else create new cluster
            newClusters.push_back(cluster);
    }
    //update vocabulary
    int idNo=0;
    for (auto & cluster : newClusters){
        ViewIndependentPart part;
        part.layerId=4;
        part.id = idNo;
        for (auto & partId : cluster){
            part.group.push_back(partId);
            hierarchy.interpreter[partId]=idNo;
        }
        vocabulary.push_back(part);
        idNo++;
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
bool PartSelectorMean::isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter){
    for (std::vector< std::set<int>>::iterator cluster = clusters.begin(); cluster!=clusters.end(); cluster++){
        auto iter1 = std::find ((*cluster).begin(), (*cluster).end(), id);
        if (iter1!=(*cluster).end()){
            iter = cluster;
            return true;
        }
    }
    return false;
}


hop3d::PartSelector* hop3d::createPartSelectorMean(void) {
    selector.reset(new PartSelectorMean());
    return selector.get();
}

hop3d::PartSelector* hop3d::createPartSelectorMean(std::string config) {
    selector.reset(new PartSelectorMean(config));
    return selector.get();
}
