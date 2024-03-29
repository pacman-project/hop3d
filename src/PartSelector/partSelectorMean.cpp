#include "hop3d/PartSelector/partSelectorMean.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
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
    useCompressionRateVD.resize(layersNo);
    clustersNoVD.resize(layersNo);
    compressionRateVD.resize(layersNo);
    for (int i=0;i<layersNo;i++){
        std::string layerName = "layerVD" + std::to_string(i+1);
        bool ucr;
        group->FirstChildElement( layerName.c_str() )->QueryBoolAttribute("useCompressionRate", &ucr);
        useCompressionRateVD[i]=ucr;
        group->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("compressionRate", &compressionRateVD[i]);
        group->FirstChildElement( layerName.c_str() )->QueryIntAttribute("clusters", &clustersNoVD[i]);
    }
    useCompressionRateVolumetric.resize(layersNo);
    clustersNoVolumetric.resize(layersNo);
    compressionRateVolumetric.resize(layersNo);
    for (int i=0;i<layersNo;i++){
        std::string layerName = "layer" + std::to_string(i+1);
        bool ucr;
        group->FirstChildElement( layerName.c_str() )->QueryBoolAttribute("useCompressionRate", &ucr);
        useCompressionRateVolumetric[i]=ucr;
        group->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("compressionRate", &compressionRateVolumetric[i]);
        group->FirstChildElement( layerName.c_str() )->QueryIntAttribute("clusters", &clustersNoVolumetric[i]);
    }

    std::string GICPConfig = (group->FirstChildElement( "GICP" )->Attribute( "configFilename" ));
    size_t found = configFilename.find_last_of("/\\");
    std::string prefix = configFilename.substr(0,found+1);
    GICPConfig = prefix+GICPConfig;
    tinyxml2::XMLDocument configGICPxml;
    configGICPxml.LoadFile(GICPConfig.c_str());
    if (configGICPxml.ErrorID())
        throw std::runtime_error("unable to load Object Composition octree config file: " + filename);
    tinyxml2::XMLElement * groupGICP = configGICPxml.FirstChildElement( "GICP" );

    groupGICP->QueryIntAttribute("verbose", &configGICP.verbose);
    groupGICP->QueryIntAttribute("guessesNo", &configGICP.guessesNo);
    groupGICP->QueryIntAttribute("maxIterations", &configGICP.maxIterations);
    groupGICP->QueryDoubleAttribute("transformationEpsilon", &configGICP.transformationEpsilon);
    groupGICP->QueryDoubleAttribute("EuclideanFitnessEpsilon", &configGICP.EuclideanFitnessEpsilon);
    groupGICP->QueryDoubleAttribute("correspondenceDist", &configGICP.correspondenceDist);
    groupGICP->QueryDoubleAttribute("alphaMin", &configGICP.alpha.first);
    groupGICP->QueryDoubleAttribute("alphaMax", &configGICP.alpha.second);
    groupGICP->QueryDoubleAttribute("betaMin", &configGICP.beta.first);
    groupGICP->QueryDoubleAttribute("betaMax", &configGICP.beta.second);
    groupGICP->QueryDoubleAttribute("gammaMin", &configGICP.gamma.first);
    groupGICP->QueryDoubleAttribute("gammaMax", &configGICP.gamma.second);
}

/// Select parts from the initial vocabulary
void PartSelectorMean::selectParts(ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo){
    int clustersNo;
    if (config.useCompressionRateVolumetric[layerNo])
        clustersNo = (int)(config.compressionRateVolumetric[layerNo]*(int)dictionary.size());
    else
        clustersNo = config.clustersNoVolumetric[layerNo];
    if ((size_t)clustersNo > dictionary.size())
        clustersNo = (int)dictionary.size();
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
    std::vector<ViewIndependentPart::Seq> clusters(clustersNo);
    std::vector<std::vector<Mat34>> offsets(clustersNo);
    for (int i=0;i<config.maxIter;i++){
        if (config.verbose==1){
            if ((config.maxIter>1)&&i%((config.maxIter+1)/10)==0){
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
        fit2clusters(centroids, dictionary, clusters, hierarchy, offsets);
        std::vector<int> oldCentroids(centroids);
        computeCentroids(clusters, centroids, dictionary, hierarchy, offsets);
        if (oldCentroids==centroids){
            if (config.verbose>0)
                std::cout << "Nothing has changed. Finish clustering after " << i+1 << " iterations\n";
            i=config.maxIter;
        }
        if (config.verbose==2){
            std::cout << "centroids new: ";
            for (size_t j=0;j<centroids.size();j++){
                std::cout << dictionary[centroids[j]].id << ", ";
            }
            std::cout << "\n";
        }
    }
    ViewIndependentPart::Seq newDictionary;
    //dictionary.clear();
    int centroidNo=0;
    for (auto &cluster : clusters){
        if (cluster.size()>0){
            ViewIndependentPart part = dictionary[centroids[centroidNo]];
            int partNo = 0;
            for (auto &partC : cluster){
                partC.offset = offsets[centroidNo][partNo];
                part.group.push_back(partC);
                partNo++;
            }
            newDictionary.push_back(part);
            centroidNo++;
        }
    }
    dictionary = newDictionary;
}

/// Select parts from the initial vocabulary
void PartSelectorMean::selectParts(ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, int layerNo){
    int clustersNo;
    if (config.useCompressionRateVD[layerNo])
        clustersNo = (int)(config.compressionRateVD[layerNo]*(int)dictionary.size());
    else
        clustersNo = config.clustersNoVD[layerNo];
    if ((size_t)clustersNo > dictionary.size())
        clustersNo = (int)dictionary.size();
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
        std::vector<int> oldCentroids(centroids);
        computeCentroids(clusters, centroids, dictionary, hierarchy);
        if (oldCentroids==centroids){
            if (config.verbose>0)
                std::cout << "Nothing has changed. Finish clustering after " << i+1 << " iterations\n";
            i=config.maxIter;
        }
        if (config.verbose==2){
            std::cout << "centroids new: ";
            for (size_t j=0;j<centroids.size();j++){
                std::cout << dictionary[centroids[j]].id << ", ";
            }
            std::cout << "\n";
        }
    }
    ViewDependentPart::Seq newDictionary;
    //dictionary.clear();
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
void PartSelectorMean::fit2clusters(const std::vector<int>& centroids, const ViewIndependentPart::Seq& dictionary, std::vector<ViewIndependentPart::Seq>& clusters, const Hierarchy& hierarchy, std::vector<std::vector<Mat34>>& offsets){
    for (size_t i=0;i<clusters.size();i++){
        clusters[i].clear();
        offsets[i].clear();
    }
    for (auto it = dictionary.begin();it!=dictionary.end();it++){// for each part
        double minDist= std::numeric_limits<double>::max();
        int clusterNo = 0;
        int centroidId = 0;
        Mat34 offsetMin;
        for (auto itCentr = centroids.begin();itCentr!=centroids.end();itCentr++){//for each cluster
            double dist = 0;
            Mat34 offset;
            if (it->layerId>2){//compute distance from centroid
                //dist = pow(1+fabs(double(it->cloud.size())-double(dictionary[*itCentr].cloud.size())),2.0)*ViewIndependentPart::distanceGICP(*it, dictionary[*itCentr],config.configGICP, offset);
                if (it->layerId==3)
                    dist += ViewIndependentPart::distanceUmeyama(*it, dictionary[*itCentr], config.distanceMetric, offset);
                else if (it->layerId==4)
                    dist += ViewIndependentPart::distanceUmeyama(*it, dictionary[*itCentr], config.distanceMetric, hierarchy.viewIndependentLayers[0], offset);
                //dist = ViewIndependentPart::distanceGICP(*it, dictionary[*itCentr],config.configGICP, offset);
            }
            /*if (it->layerId==5){//compute distance from centroid
                dist = ViewIndependentPart::distance(*it, dictionary[*itCentr], offset);
            }
            else if (it->layerId==6){//compute distance from centroid
                dist = ViewIndependentPart::distance(*it, dictionary[*itCentr], hierarchy.viewIndependentLayers[1], offset);
            }
            else if (it->layerId==7){//compute distance from centroid
                dist = ViewIndependentPart::distance(*it, dictionary[*itCentr], hierarchy.viewIndependentLayers[1], hierarchy.viewIndependentLayers[2], offset);
            }*/
            if (dist<minDist){
                minDist = dist;
                centroidId = clusterNo;
                offsetMin = offset;
            }
            clusterNo++;
        }
        clusters[centroidId].push_back(*it);
        offsets[centroidId].push_back(offsetMin);
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

/// assign parts to clusters according to given centroid
void PartSelectorMean::fit2clusters(const std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<ViewDependentPart::Seq>& clusters){
    for (size_t i=0;i<clusters.size();i++)
        clusters[i].clear();
    for (auto it = dictionary.begin();it!=dictionary.end();it++){// for each part
        double minDist= std::numeric_limits<double>::max();
        int clusterNo = 0;
        int centroidId = 0;
        for (auto itCentr = centroids.begin();itCentr!=centroids.end();itCentr++){//for each cluster
            double dist = 0;
            int rotIdx;
            if (it->layerId==2){//compute distance from centroid
                if (config.distanceMetric==3){
                    Mat34 transform;
                    dist=ViewDependentPart::distanceInvariant(*it,dictionary[*itCentr], 3, transform, rotIdx);
                }
                else
                    dist = ViewDependentPart::distance(*it,dictionary[*itCentr],hierarchy.firstLayer, config.distanceMetric);
            }
            else if (it->layerId==3){//compute distance from centroid
                if (config.distanceMetric==3){
                    Mat34 transform;
                    dist=ViewDependentPart::distanceInvariant(*it,dictionary[*itCentr], 3, transform, rotIdx);
                }
                else
                    dist = ViewDependentPart::distance(*it,dictionary[*itCentr], hierarchy.viewDependentLayers[0], hierarchy.firstLayer, config.distanceMetric);
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
        std::cout << "Elements no in clusters (clusters size: " << clusters.size() << "): ";
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
void PartSelectorMean::computeCentroids(const std::vector<ViewIndependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewIndependentPart::Seq& dictionary, const Hierarchy& hierarchy, std::vector<std::vector<Mat34>>& offsets){
    int clusterNo=0;
    for (auto itClust = clusters.begin(); itClust!=clusters.end();itClust++){ //for each cluster
        double distMin = std::numeric_limits<double>::max();
        int centerId=0;
        for (auto itPart = itClust->begin(); itPart!=itClust->end();itPart++){//for each part in cluster
            double distSum = 0; //compute new centroid
            std::vector<Mat34> offsetsTmp;
            for (auto itPart2 = itClust->begin(); itPart2!=itClust->end();itPart2++){//compute mean dist for each part as a centroid
                Mat34 offset;
                if (itPart->layerId>2){//compute distance from centroid
                    //distSum += pow(1+fabs(double(itPart->cloud.size())-double(itPart2->cloud.size())),2.0)*ViewIndependentPart::distanceGICP(*itPart, *itPart2, config.configGICP, offset);
                    if (itPart->layerId==3)
                        distSum += ViewIndependentPart::distanceUmeyama(*itPart, *itPart2, config.distanceMetric, offset);
                    else if (itPart->layerId==4)
                        distSum += ViewIndependentPart::distanceUmeyama(*itPart, *itPart2, config.distanceMetric, hierarchy.viewIndependentLayers[0], offset);
                    //distSum += ViewIndependentPart::distanceGICP(*itPart, *itPart2, config.configGICP, offset);
                    offsetsTmp.push_back(offset);
                }
                /*if (itPart->layerId==5){//compute distance from centroid
                    distSum += ViewIndependentPart::distance(*itPart, *itPart2, offset);
                }
                else if (itPart->layerId==6){//compute distance from centroid
                    distSum += ViewIndependentPart::distance(*itPart, *itPart2, hierarchy.viewIndependentLayers[1], offset);
                }
                else if (itPart->layerId==7){//compute distance from centroid
                    distSum += ViewIndependentPart::distance(*itPart, *itPart2, hierarchy.viewIndependentLayers[1], hierarchy.viewIndependentLayers[2], offset);
                }*/
            }
            if (distSum<distMin){
                distMin=distSum;
                centerId=itPart->id;
                offsets[clusterNo] = offsetsTmp;
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

///find new center of cluster
int PartSelectorMean::centerOfCluster(const std::set<int>& cluster, const ViewDependentPart::Seq& vocabulary, const Hierarchy& hierarchy) const{
    double distMin = std::numeric_limits<double>::max();
    int centerId=0;
    for (auto& id : cluster){//for each part id in cluster
        double distSum = 0; //compute new centroid
        for (auto& id2 : cluster){//compute mean dist for each part as a centroid
            double dist=0;
            int rotIdx;
            if (vocabulary[id].layerId==2){
                if (config.distanceMetric==3){
                    Mat34 transform;
                    dist=ViewDependentPart::distanceInvariant(vocabulary[id],vocabulary[id2], 3, transform, rotIdx);
                }
                else
                    dist=ViewDependentPart::distance(vocabulary[id],vocabulary[id2],hierarchy.firstLayer, config.distanceMetric);
            }
            else if (vocabulary[id].layerId==3){
                if (config.distanceMetric==3){
                    Mat34 transform;
                    dist=ViewDependentPart::distanceInvariant(vocabulary[id],vocabulary[id2], 3, hierarchy.viewDependentLayers[0], transform, rotIdx);
                }
                else
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

/// compute centroids for given clusters
void PartSelectorMean::computeCentroids(const std::vector<ViewDependentPart::Seq>& clusters, std::vector<int>& centroids, const ViewDependentPart::Seq& dictionary, const Hierarchy& hierarchy){
    int clusterNo=0;
    for (auto& cluster : clusters){ //for each cluster
        double distMin = std::numeric_limits<double>::max();
        int centerId=0;
        for (auto itPart = cluster.begin(); itPart!=cluster.end();itPart++){//for each part in cluster
            double distSum = 0; //compute new centroid
            for (auto itPart2 = cluster.begin(); itPart2!=cluster.end();itPart2++){//compute mean dist for each part as a centroid
                double dist=0;
                int rotIdx;
                if (itPart->layerId==2){
                    if (config.distanceMetric==3){
                        Mat34 transform;
                        dist=ViewDependentPart::distanceInvariant(*itPart,*itPart2, 3, transform, rotIdx);
                    }
                    else
                        dist=ViewDependentPart::distance(*itPart,*itPart2,hierarchy.firstLayer, config.distanceMetric);
                }
                else if (itPart->layerId==3){
                    if (config.distanceMetric==3){
                        Mat34 transform;
                        dist=ViewDependentPart::distanceInvariant(*itPart,*itPart2, 3, transform, rotIdx);
                    }
                    else
                        dist=ViewDependentPart::distance(*itPart,*itPart2, hierarchy.viewDependentLayers[0], hierarchy.firstLayer, config.distanceMetric);
                }
                distSum+=dist;
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
            if (i==dictionary.size()-1){
                std::cout << "dictionary.size() " << dictionary.size() << "\n";
                std::cout << "centerId3 " << centerId  << " does not exist \n";
                std::cout << "parts in dictionary: ";
                for (auto& part : dictionary){
                    std::cout << part.id << " ";
                }
                std::cout << "\n";
                std::cout << "Something is wrong with ids in clusterization\n";
                getchar();
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
