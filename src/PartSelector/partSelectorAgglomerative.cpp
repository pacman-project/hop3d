#include "hop3d/PartSelector/partSelectorAgglomerative.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include <ctime>

using namespace hop3d;

/// A single instance of part selector
PartSelectorAgglomerative::Ptr selectorAgg;

PartSelectorAgglomerative::PartSelectorAgglomerative(void) : PartSelector("Hierarchical agglomerative Part Selector", SELECTOR_AGGLOMERATIVE) {
}

/// Construction
PartSelectorAgglomerative::PartSelectorAgglomerative(std::string config) :
        PartSelector("Hierarchical agglomerative Part Selector", SELECTOR_AGGLOMERATIVE), config(config) {
}

/// Destruction
PartSelectorAgglomerative::~PartSelectorAgglomerative(void) {
}

///config class constructor
PartSelectorAgglomerative::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load part selector config file: " + filename);
    tinyxml2::XMLElement * group = config.FirstChildElement( "PartSelector" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load part selector parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryIntAttribute("maxIter", &maxIter);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("distanceMetric", &distanceMetric);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("layersNo", &layersNo);
    maxDist.resize(layersNo);
    maxClusterDist.resize(layersNo);
    for (int i=0;i<layersNo;i++){
        std::string layerName = "layer" + std::to_string(i+1);
        group->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("maxDist", &maxDist[i]);
        group->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("maxClusterDist", &maxClusterDist[i]);
    }
    group->FirstChildElement( "GICP" )->QueryIntAttribute("verbose", &configGICP.verbose);
    group->FirstChildElement( "GICP" )->QueryIntAttribute("guessesNo", &configGICP.guessesNo);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("correspondenceDist", &configGICP.correspondenceDist);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("alphaMin", &configGICP.alpha.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("alphaMax", &configGICP.alpha.second);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("betaMin", &configGICP.beta.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("betaMax", &configGICP.beta.second);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("gammaMin", &configGICP.gamma.first);
    group->FirstChildElement( "GICP" )->QueryDoubleAttribute("gammaMax", &configGICP.gamma.second);
}

/// Select parts from the initial vocabulary
void PartSelectorAgglomerative::selectParts(ViewIndependentPart::Seq& dictionary, int layerNo){
    std::vector<std::vector<double>> distanceMatrix(dictionary.size(), std::vector<double>(dictionary.size()));
    std::vector<std::vector<Mat34>> transformMatrix(dictionary.size(), std::vector<Mat34>(dictionary.size()));
    std::cout << "compute distance matrix...";
    computeDistanceMatrix(dictionary, distanceMatrix, transformMatrix);
    if (config.verbose>0){
        std::cout << "done\n";
        std::cout << "Initial number of words in dictionary: " << dictionary.size() << "\n";
        std::cout << "Max dist: " << config.maxDist[layerNo-1] << "\n";
    }
    std::vector<std::vector<int>> clusters(dictionary.size());
    std::vector<int> centroids(dictionary.size());//index in dictionary of part assigned to the centroid;
    for (size_t i=0;i<clusters.size();i++){//assign centers of centroid
        clusters[i].push_back((int)i);
        centroids[i]=(int)i;
    }
    for (size_t i=0;i<dictionary.size()*dictionary.size();i++){
        if (config.verbose==1){
            if (((dictionary.size()*dictionary.size()+1)>10)&&i%((dictionary.size()*dictionary.size()+1)/10)==0){
                std::cout << "Iteration: " << i+1 << "/" << dictionary.size()*dictionary.size() << " clusters no: " << clusters.size() << "\n";
            }
        }
        std::pair<int,int> pairedIds;
        double minDist = findMinDistance(distanceMatrix, pairedIds);
        //std::cout << "find min dist between " << pairedIds.first << "->" << pairedIds.second << "\n";
        distanceMatrix[pairedIds.first][pairedIds.second] = -1;
  //      bool finishClusterization(false);
        std::cout << "dist " << minDist << " max dist " << config.maxDist[layerNo-1] << "\n";
        if (minDist>=config.maxDist[layerNo-1]||clusters.size()==1)
            break;
            //finishClusterization = true;
        //merge two centroids
        std::pair<int,int> clustersIds;
        findPartsInClusters(clusters, pairedIds, clustersIds);
        //std::cout << pairedIds.first << " is in cluster " << clustersIds.first << "\n";
        //std::cout << pairedIds.second << " is in cluster " << clustersIds.second << "\n";
        if (clustersIds.first!=clustersIds.second){
            if (mergeTwoClusters(clusters, centroids, clustersIds, distanceMatrix, layerNo))
                updateCentroids(clusters, distanceMatrix, centroids, clustersIds);
            else {
                reduceEntropy(clusters, centroids, clustersIds, distanceMatrix);
                computeCentroid(clusters[clustersIds.first], distanceMatrix, centroids[clustersIds.first]);
                computeCentroid(clusters[clustersIds.second], distanceMatrix, centroids[clustersIds.second]);
            }
        }
        if (config.verbose==2){
            std::cout << "Elements no in clusters (clusters size: " << clusters.size() << "): ";
            for (size_t i=0;i<clusters.size();i++)
                std::cout << i << ":" << clusters[i].size() << ", ";
            std::cout << "\nClusters: ";
            for (size_t i=0;i<clusters.size();i++){
                std::cout <<  i << ":{";
                for (size_t j=0;j<clusters[i].size();j++)
                    std::cout << clusters[i][j] << ", ";
                std::cout << "},";
            }
            std::cout << "\n";
        }
    }

    /// generate new dictionary
    ViewIndependentPart::Seq newDictionary;
    int centroidNo=0;
    for (const auto &cluster : clusters){
        if (cluster.size()>0){
            ViewIndependentPart part = dictionary[centroids[centroidNo]];
            for (const auto &partId : cluster){
                part.group.push_back(dictionary[partId]);
                if (centroids[centroidNo] == partId)
                    part.group.back().offset = Mat34::Identity();
                else
                    part.group.back().offset = transformMatrix[centroids[centroidNo]][partId];
            }
            newDictionary.push_back(part);
            centroidNo++;
        }
    }
    dictionary = newDictionary;
}

/// compute distance matrix
void PartSelectorAgglomerative::computeDistanceMatrix(const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<std::vector<double>>& distanceMatrix, std::vector<std::vector<Mat34>>& transformMatrix) const{
    for (size_t idA=0;idA<dictionary.size();idA++){
        for (size_t idB=idA;idB<dictionary.size();idB++){
            if (idA==idB){
                distanceMatrix[idB][idA]=0;
                transformMatrix[idA][idB] = Mat34::Identity();
            }
            else{
                double dist(0); Mat34 transform;
                if (dictionary[idA].layerId==2){//compute distance from centroid
                    if (config.distanceMetric==3){
                        dist=ViewDependentPart::distanceInvariant(dictionary[idA], dictionary[idB], 3, transform);
                    }
                    else
                        dist = ViewDependentPart::distance(dictionary[idA], dictionary[idB], hierarchy.firstLayer, config.distanceMetric);
                }
                else if (dictionary[idA].layerId==3){//compute distance from centroid
                    if (config.distanceMetric==3){
                        dist=ViewDependentPart::distanceInvariant(dictionary[idA], dictionary[idB], 3, transform);
                    }
                    else
                        dist = ViewDependentPart::distance(dictionary[idA], dictionary[idB], hierarchy.viewDependentLayers[0], hierarchy.firstLayer, config.distanceMetric);
                }
                distanceMatrix[idA][idB]=dist;
                distanceMatrix[idB][idA]=dist;
                transformMatrix[idA][idB] = transform;
                transformMatrix[idB][idA] = transform;
            }
        }
    }
}

/// compute distance matrix for view-independent parts
void PartSelectorAgglomerative::computeDistanceMatrix(const ViewIndependentPart::Seq& dictionary, std::vector<std::vector<double>>& distanceMatrix, std::vector<std::vector<Mat34>>& transformMatrix) const{
    for (size_t idA=0;idA<dictionary.size();idA++){
        for (size_t idB=idA+1;idB<dictionary.size();idB++){
            double dist(0); Mat34 transform;
            if (dictionary[idA].layerId>2){//compute distance from centroid
                dist = (1+fabs(double(dictionary[idA].cloud.size())-double(dictionary[idB].cloud.size())))*ViewIndependentPart::distanceGICP(dictionary[idA], dictionary[idB], config.configGICP, transform);
            }
            distanceMatrix[idA][idB]=dist;
            distanceMatrix[idB][idA]=dist;
            transformMatrix[idA][idB] = transform;
            transformMatrix[idB][idA] = transform.inverse();
        }
    }
}

/// find min distance int the distance matrix
double PartSelectorAgglomerative::findMinDistance(const std::vector<std::vector<double>>& distanceMatrix, std::pair<int,int>& pairedIds) const{
    double minDist=std::numeric_limits<double>::max();
    for (size_t idA=0;idA<distanceMatrix.size();idA++){
        for (size_t idB=idA+1;idB<distanceMatrix.size();idB++){
            if ((distanceMatrix[idA][idB]>=0)&&(distanceMatrix[idA][idB]<minDist)){
                minDist=distanceMatrix[idA][idB];
                pairedIds = std::make_pair(idA,idB);
            }
        }
    }
    return minDist;
}

/// find clusters ids to which contain specyfic parts (pairedIds)
void PartSelectorAgglomerative::findPartsInClusters(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& pairedIds, std::pair<int,int>& clustersIds) const{
    bool found[2]={false, false};
    for (size_t i=0;i<clusters.size();i++){
        for (auto &id : clusters[i]){
            if (id==pairedIds.first){
                clustersIds.first = (int)i;
                found[0]=true;
            }
            if (id==pairedIds.second){
                clustersIds.second = (int)i;
                found[1]=true;
            }
            if (found[0]&&found[1]) break;
        }
        if (found[0]&&found[1]) break;
    }
}

/// compute max distance between centroid of the first cluster and all parts in the second cluster
double PartSelectorAgglomerative::computeMaxDist(const std::vector<std::vector<int>>& clusters, const std::pair<int,int>& clustersIds, const std::vector<int>& centroids, const std::vector<std::vector<double>>& distanceMatrix) const{
    int partIdA = centroids[clustersIds.first];
    double maxDist = std::numeric_limits<double>::min();
    for (const auto &partIdB : clusters[clustersIds.second]){
        double dist;
        if (partIdA>partIdB)//because up-triangle elements in distance matrix are cleaned
            dist = distanceMatrix[partIdA][partIdB];
        else
            dist = distanceMatrix[partIdB][partIdA];
        if (dist>maxDist){
            maxDist = dist;
        }
    }
    return maxDist;
}

/// merge two clusters
bool PartSelectorAgglomerative::mergeTwoClusters(std::vector<std::vector<int>>& clusters, const std::vector<int>& centroids, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix, int layerNo) const{
    double maxDist = computeMaxDist(clusters, clustersIds, centroids, distanceMatrix);
    if (maxDist<config.maxClusterDist[layerNo-1]){
        if (clustersIds.first<clustersIds.second){
            // merge clusters
            clusters[clustersIds.first].insert(clusters[clustersIds.first].end(), clusters[clustersIds.second].begin(), clusters[clustersIds.second].end());
            // remove second cluster
            clusters.erase(clusters.begin()+clustersIds.second);
        }
        else{
            // merge clusters
            clusters[clustersIds.second].insert(clusters[clustersIds.second].end(), clusters[clustersIds.first].begin(), clusters[clustersIds.first].end());
            // remove second cluster
            clusters.erase(clusters.begin()+clustersIds.first);
        }
        return true;
    }
    else
        return false;
}

/// update centroids
void PartSelectorAgglomerative::updateCentroids(const std::vector<std::vector<int>>& clusters, const std::vector<std::vector<double>>& distanceMatrix, std::vector<int>& centroids, const std::pair<int,int>& clustersIds) const{
    if (clustersIds.first<clustersIds.second){
        // compute new first cetroid
        computeCentroid(clusters[clustersIds.first], distanceMatrix, centroids[clustersIds.first]);
        // remove second centroid
        //std::cout << "remove centroid " << clustersIds.second << "\n";
        centroids.erase(centroids.begin()+clustersIds.second);
    }
    else{
        // compute new first cetroid
        computeCentroid(clusters[clustersIds.second], distanceMatrix, centroids[clustersIds.second]);
        // remove second centroid
        //std::cout << "remove centroid " << clustersIds.first << "\n";
        centroids.erase(centroids.begin()+clustersIds.first);
    }
}

/// modify clusters according to min distance
void PartSelectorAgglomerative::reduceEntropy(std::vector<std::vector<int>>& clusters, const std::vector<int>& centroids, const std::pair<int,int>& clustersIds, const std::vector<std::vector<double>>& distanceMatrix){
    int centroidIdA = centroids[clustersIds.first];
    int centroidIdB = centroids[clustersIds.second];
    //std::cout << "centroid A " << centroidIdA << "\n";
    //std::cout << "centroid B " << centroidIdB << "\n";
    std::set<int> ids2move;
    int partId=0;
    /*std::cout << "cluster A before ";
    for (const auto &partIdB : clusters[clustersIds.first]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";
    std::cout << "cluster B before ";
    for (const auto &partIdB : clusters[clustersIds.second]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";*/
    for (const auto &partIdB : clusters[clustersIds.second]){
        double dist2centroidA, dist2centroidB;
        if (partIdB>centroidIdA)//because up-triangle elements in distance matrix are cleaned
            dist2centroidA = distanceMatrix[partIdB][centroidIdA];
        else
            dist2centroidA = distanceMatrix[centroidIdA][partIdB];
        if (partIdB>centroidIdB)//because up-triangle elements in distance matrix are cleaned
            dist2centroidB = distanceMatrix[partIdB][centroidIdB];
        else
            dist2centroidB = distanceMatrix[centroidIdB][partIdB];
        //std::cout << "dist 2 centroid A " << partIdB << "->" << centroidIdA << " = " << dist2centroidA << "\n";
        //std::cout << "dist 2 centroid B " << partIdB << "->" << centroidIdB << " = " << dist2centroidB << "\n";
        if (dist2centroidB>dist2centroidA){
            ids2move.insert(partId);
        }
        partId++;
    }
    for (auto rit=ids2move.rbegin(); rit != ids2move.rend(); ++rit){
        clusters[clustersIds.first].push_back(clusters[clustersIds.second][*rit]);
        clusters[clustersIds.second].erase(clusters[clustersIds.second].begin()+*rit);
    }
    ids2move.clear();
    /*std::cout << "cluster A middle ";
    for (const auto &partIdB : clusters[clustersIds.first]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";
    std::cout << "cluster B middle ";
    for (const auto &partIdB : clusters[clustersIds.second]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";*/
    partId=0;
    for (const auto &partIdA : clusters[clustersIds.first]){
        double dist2centroidA, dist2centroidB;
        if (partIdA>centroidIdA)//because up-triangle elements in distance matrix are cleaned
            dist2centroidA = distanceMatrix[partIdA][centroidIdA];
        else
            dist2centroidA = distanceMatrix[centroidIdA][partIdA];
        if (partIdA>centroidIdB)//because up-triangle elements in distance matrix are cleaned
            dist2centroidB = distanceMatrix[partIdA][centroidIdB];
        else
            dist2centroidB = distanceMatrix[centroidIdB][partIdA];
        //std::cout << "dist 2 centroid A " << partIdA << "->" << centroidIdA << " = " << dist2centroidA << "\n";
        //std::cout << "dist 2 centroid B " << partIdA << "->" << centroidIdB << " = " << dist2centroidB << "\n";
        if (dist2centroidB<dist2centroidA){
            ids2move.insert(partId);
        }
        partId++;
    }
    for (auto rit=ids2move.rbegin(); rit != ids2move.rend(); ++rit){
        clusters[clustersIds.second].push_back(clusters[clustersIds.first][*rit]);
        clusters[clustersIds.first].erase(clusters[clustersIds.first].begin()+*rit);
    }
    /*std::cout << "cluster A after ";
    for (const auto &partIdB : clusters[clustersIds.first]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";
    std::cout << "cluster B after ";
    for (const auto &partIdB : clusters[clustersIds.second]){
        std::cout << partIdB << ", ";
    }
    std::cout << "\n";*/
}

/// compute centroids using pre-computed distance in distance matrix
void PartSelectorAgglomerative::computeCentroid(const std::vector<int>& cluster, const std::vector<std::vector<double>>& distanceMatrix, int& centroid) const{
    double minDist = std::numeric_limits<double>::max();
    centroid=cluster.front();
    for (const auto &partIdA : cluster){
        double sumDist = 0;
        for (const auto &partIdB : cluster){
            double dist;
            if (partIdA>partIdB)//because up-triangle elements in distance matrix are cleaned
                dist = distanceMatrix[partIdA][partIdB];
            else
                dist = distanceMatrix[partIdB][partIdA];
            sumDist+=dist;
        }
        if (sumDist<minDist){
            minDist = sumDist;
            centroid = partIdA;
        }
    }
}

/// compute centroids using pre-computed distance in distance matrix
void PartSelectorAgglomerative::computeCentroids(const std::vector<std::vector<int>>& clusters, const std::vector<std::vector<double>>& distanceMatrix, std::vector<int>& centroids) const{
    centroids.clear();
    centroids.reserve(clusters.size());
    for (const auto &cluster : clusters){
        int centerId;
        computeCentroid(cluster, distanceMatrix, centerId);
        centroids.push_back(centerId);
    }
}

/// Select parts from the initial vocabulary
void PartSelectorAgglomerative::selectParts(ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo){
    std::vector<std::vector<double>> distanceMatrix(dictionary.size(), std::vector<double>(dictionary.size()));
    std::vector<std::vector<Mat34>> transformMatrix(dictionary.size(), std::vector<Mat34>(dictionary.size()));
    std::cout << "compute distance matrix...";
    computeDistanceMatrix(dictionary, hierarchy, distanceMatrix, transformMatrix);
    if (config.verbose>0){
        std::cout << "done\n";
        std::cout << "Initial number of words in dictionary: " << dictionary.size() << "\n";
        std::cout << "Max dist: " << config.maxDist[layerNo-1] << "\n";
    }
    std::vector<std::vector<int>> clusters(dictionary.size());
    std::vector<int> centroids(dictionary.size());//index in dictionary of part assigned to the centroid;
    for (size_t i=0;i<clusters.size();i++){//assign centers of centroid
        clusters[i].push_back((int)i);
        centroids[i]=(int)i;
    }
    for (size_t i=0;i<dictionary.size()*dictionary.size();i++){
        if (config.verbose==1){
            if ((dictionary.size()>1)&&i%((dictionary.size()*dictionary.size()+1)/10)==0){
                std::cout << "Iteration: " << i+1 << "/" << dictionary.size()*dictionary.size() << " clusters no: " << clusters.size() << "\n";
            }
        }
        std::pair<int,int> pairedIds;
        double minDist = findMinDistance(distanceMatrix, pairedIds);
        //std::cout << "find min dist between " << pairedIds.first << "->" << pairedIds.second << "\n";
        distanceMatrix[pairedIds.first][pairedIds.second] = -1;
        //std::cout << "dist " << minDist << " max dist " << config.maxDist[layerNo-1] << "\n";
        if (minDist>=config.maxDist[layerNo-1])
            break;
        //merge two centroids
        std::pair<int,int> clustersIds;
        findPartsInClusters(clusters, pairedIds, clustersIds);
        //std::cout << pairedIds.first << " is in cluster " << clustersIds.first << "\n";
        //std::cout << pairedIds.second << " is in cluster " << clustersIds.second << "\n";
        if (clustersIds.first!=clustersIds.second){
            if (mergeTwoClusters(clusters, centroids, clustersIds, distanceMatrix, layerNo))
                updateCentroids(clusters, distanceMatrix, centroids, clustersIds);
            else {
                reduceEntropy(clusters, centroids, clustersIds, distanceMatrix);
                computeCentroid(clusters[clustersIds.first], distanceMatrix, centroids[clustersIds.first]);
                computeCentroid(clusters[clustersIds.second], distanceMatrix, centroids[clustersIds.second]);
            }
        }
        if (config.verbose==2){
            std::cout << "Elements no in clusters (clusters size: " << clusters.size() << "): ";
            for (size_t i=0;i<clusters.size();i++)
                std::cout << i << ":" << clusters[i].size() << ", ";
            std::cout << "\nClusters: ";
            for (size_t i=0;i<clusters.size();i++){
                std::cout <<  i << ":{";
                for (size_t j=0;j<clusters[i].size();j++)
                    std::cout << clusters[i][j] << ", ";
                std::cout << "},";
            }
            std::cout << "\n";
            std::cout << "\nCentroids: ";
            for (auto &centroid : centroids){
                std::cout <<  centroid << ", ";
            }
            std::cout << "\n";
        }
    }
    //compute centroids for each cluster
    //computeCentroids(clusters, distanceMatrix, centroids);

    /// generate new dictionary
    ViewDependentPart::Seq newDictionary;
    int centroidNo=0;
    for (const auto &cluster : clusters){
        if (cluster.size()>0){
            ViewDependentPart part = dictionary[centroids[centroidNo]];
            for (const auto &partId : cluster){
                part.group.push_back(dictionary[partId]);
                if (centroids[centroidNo] == partId)
                    part.group.back().offset = Mat34::Identity();
                else
                    part.group.back().offset = transformMatrix[partId][centroids[centroidNo]];
            }
            newDictionary.push_back(part);
            centroidNo++;
        }
    }
    dictionary = newDictionary;
}

///find new center of cluster
int PartSelectorAgglomerative::centerOfCluster(const std::set<int>& cluster, const ViewDependentPart::Seq& vocabulary, const Hierarchy& hierarchy) const{
    double distMin = std::numeric_limits<double>::max();
    int centerId=0;
    for (auto& id : cluster){//for each part id in cluster
        double distSum = 0; //compute new centroid
        for (auto& id2 : cluster){//compute mean dist for each part as a centroid
            double dist=0;
            if (vocabulary[id].layerId==2){
                if (config.distanceMetric==3){
                    Mat34 transform;
                    dist=ViewDependentPart::distanceInvariant(vocabulary[id],vocabulary[id2], 3, transform);
                }
                else
                    dist=ViewDependentPart::distance(vocabulary[id],vocabulary[id2],hierarchy.firstLayer, config.distanceMetric);
            }
            else if (vocabulary[id].layerId==3){
                dist=ViewDependentPart::distance(vocabulary[id],vocabulary[id2], hierarchy.viewDependentLayers[0], hierarchy.firstLayer, config.distanceMetric);
            }
            distSum+=dist;
        }
        if (distSum<distMin){
            distMin=distSum;
            centerId=id;
        }
    }
    return centerId;
}

/// get clusters of parts id stored in octree (one cluster per voxel)
void PartSelectorAgglomerative::createUniqueClusters(const std::vector< std::set<int>>& clusters, std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy){
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
        //ViewDependentPart vdp = hierarchy.viewDependentLayers.back()[*cluster.begin()];//get view dependent part related to view-independent part
        //select representative part
        int clusterCenter = centerOfCluster(cluster, hierarchy.viewDependentLayers.back(), hierarchy);
        ViewDependentPart vdp = hierarchy.viewDependentLayers.back()[clusterCenter];//get view dependent part related to view-independent part
        Vec3 normal;
        hierarchy.getNormal(vdp,normal);
        //vdp.getNormal(normal,hierarchy.viewDependentLayers[0], hierarchy.firstLayer);
        part.pose = Mat34::Identity();
        part.pose = NormalImageFilter::coordinateFromNormal(normal);
        std::vector<int> clusterNoOrder; //we have to change the order in the cluster (representative part is first)
        clusterNoOrder.reserve(cluster.size());
        clusterNoOrder.push_back(clusterCenter);
        for (auto & partId : cluster)
            if (partId!=clusterCenter)
                clusterNoOrder.push_back(partId);
        for (auto & partId : clusterNoOrder){//representative part is first in the group
            ViewIndependentPart partTmp;
            vdp = hierarchy.viewDependentLayers.back()[partId];//get view dependent part related to view-independent part
            //vdp.getNormal(normal, hierarchy.viewDependentLayers[0], hierarchy.firstLayer);
            hierarchy.getNormal(vdp,normal);
            partTmp.pose = NormalImageFilter::coordinateFromNormal(normal);
            partTmp.id = partId;
            part.group.push_back(partTmp);
            hierarchy.interpreter[partId]=idNo;
        }
        vocabulary.push_back(part);
        idNo++;
    }
}

/// get clusters of parts id stored in octree (one cluster per voxel)
bool PartSelectorAgglomerative::isInOctets(std::vector< std::set<int>>& clusters, int id, std::vector< std::set<int>>::iterator& iter){
    for (std::vector< std::set<int>>::iterator cluster = clusters.begin(); cluster!=clusters.end(); cluster++){
        auto iter1 = std::find ((*cluster).begin(), (*cluster).end(), id);
        if (iter1!=(*cluster).end()){
            iter = cluster;
            return true;
        }
    }
    return false;
}

hop3d::PartSelector* hop3d::createPartSelectorAgglomerative(void) {
    selectorAgg.reset(new PartSelectorAgglomerative());
    return selectorAgg.get();
}

hop3d::PartSelector* hop3d::createPartSelectorAgglomerative(std::string config) {
    selectorAgg.reset(new PartSelectorAgglomerative(config));
    return selectorAgg.get();
}
