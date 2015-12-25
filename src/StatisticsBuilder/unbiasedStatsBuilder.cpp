#include "StatisticsBuilder/unbiasedStatsBuilder.h"

using namespace hop3d;

/// A single instance of unbiased stats builder
UnbiasedStatsBuilder::Ptr builder;

UnbiasedStatsBuilder::UnbiasedStatsBuilder(void) : StatsBuilder("Unbiased Statistics Builder", STATS_UNBIASED) {
}

/// Construction
UnbiasedStatsBuilder::UnbiasedStatsBuilder(std::string config) :
        StatsBuilder("Unbiased Statistics Builder", STATS_UNBIASED), config(config) {
}

/// Destruction
UnbiasedStatsBuilder::~UnbiasedStatsBuilder(void) {
}

///config class constructor
UnbiasedStatsBuilder::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load unbiased stats builder config file: " + filename);
	tinyxml2::XMLElement * group = config.FirstChildElement("StatsBuilder");
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    group->FirstChildElement( "parameters" )->QueryBoolAttribute("useEuclideanCoordinates", &useEuclideanCoordinates);
    if (verbose == 1) {
        std::cout << "Load statistics builder parameters...\n";
    }
}

/// compute statistics for the set of octets
void UnbiasedStatsBuilder::computeStatistics(const std::vector<Octet>& octets, int layerNo, int startId, ViewDependentPart::Seq& dictionary){
    dictionary.clear();
    std::vector<Octet::Seq> groups;
    groupElements(octets, groups);
    if (config.verbose==1){
        std::cout << "Compute Gaussians for " << groups.size() << " groups...\n";
    }
    size_t partId=startId;
    int iterNo=1;
    for (auto it = groups.begin(); it!=groups.end(); it++){ //compute statistics
        ViewDependentPart part;
        computeGaussians(*it, part);
        part.partsPosNorm = it->back().partsPosNorm;//todo compute statistic
        part.partIds=it->back().partIds;//copy octet
        part.layerId = layerNo;
        part.id = (int)partId;
        partId++;
        dictionary.push_back(part);
        if (config.verbose==1){
            if ((groups.size()>10)&&iterNo%(groups.size()/10)==0){
                std::cout << "Iteration: " << iterNo << "/" << groups.size() << "\n";
            }
        }
    }
    if (config.verbose==1){
        std::cout << "done.\n";
    }
}

/// compute statistics for the set of octets
void UnbiasedStatsBuilder::computeStatistics(const std::vector<ViewIndependentPart>& elements, int layerNo, int startId, ViewIndependentPart::Seq& dictionary){
    dictionary.clear();
    std::vector<ViewIndependentPart::Seq> groups;
    groupElements(elements, groups);
    if (config.verbose==1){
        std::cout << "Compute Gaussians for " << groups.size() << " groups...\n";
    }
    size_t partId=startId;
    int iterNo=1;
    for (auto it = groups.begin(); it!=groups.end(); it++){ //compute statistics
        ViewIndependentPart part;
        part.pose = Mat34::Identity();
        computeGaussians(*it, part);
        part.partIds=it->back().partIds;//copy octet
        part.layerId = layerNo;
        part.id = (int)partId;
        computeNeigbourPoses(part);
        partId++;
        dictionary.push_back(part);
        if (config.verbose==1){
            if ((groups.size()>10)&&iterNo%(groups.size()/10)==0){
                std::cout << "Iteration: " << iterNo << "/" << groups.size() << "\n";
            }
        }
    }
    if (config.verbose==1){
        std::cout << "done.\n";
    }
}

/// compute mean SE3 transformations from gaussians
void UnbiasedStatsBuilder::computeNeigbourPoses(ViewIndependentPart& part) const{
    for (size_t i = 0; i<part.gaussians.size(); i++){
        for (size_t j = 0; j<part.gaussians[i].size(); j++){
            for (size_t k = 0; k<part.gaussians[i][j].size(); k++){
                GaussianSE3 gauss;
                part.neighbourPoses[i][j][k]=gauss.fromVector(part.gaussians[i][j][k].mean);
            }
        }
    }
}
/// Compute Gaussians for the group of octets
void UnbiasedStatsBuilder::computeGaussians(Octet::Seq& group, ViewDependentPart& part) const{
    for (size_t i = 0; i<part.gaussians.size(); i++){
        for (size_t j = 0; j<part.gaussians[i].size(); j++){
            if (i==1&&j==1){
                part.gaussians[i][j].mean = Vec3(0,0,0);
                part.gaussians[i][j].covariance = Mat33::Identity();
            }
            else{
                if (group.back().partIds[i][j]!=-1){
                    Gaussian3D gauss;
                    computeGaussian(group, gauss, (unsigned int)i, (unsigned int)j);
                    part.gaussians[i][j].mean = gauss.mean;
                    part.gaussians[i][j].covariance = gauss.covariance;
                }
            }
        }
    }
}

/// Compute Gaussians for the group of view independent parts
void UnbiasedStatsBuilder::computeGaussians(ViewIndependentPart::Seq& group, ViewIndependentPart& part) const{
    for (size_t i = 0; i<part.gaussians.size(); i++){
        for (size_t j = 0; j<part.gaussians[i].size(); j++){
            for (size_t k = 0; k<part.gaussians[i][j].size(); k++){
                if (group.back().partIds[i][j][k]!=-1){
                    GaussianSE3 gauss;
                    computeGaussian(group, gauss, (unsigned int)i, (unsigned int)j, (unsigned int)k);
                    part.gaussians[i][j][k].mean = gauss.mean;
                    part.gaussians[i][j][k].covariance = gauss.covariance;
                }
            }
        }
    }
}

/// compute Gaussian parameters
void UnbiasedStatsBuilder::computeGaussian(const Octet::Seq& group, Gaussian3D& gauss, unsigned int u, unsigned int v) const{
    Vec3 mean(0,0,0);
    for (auto& octet : group){
        if (config.useEuclideanCoordinates){
            mean+=octet.partsPosEucl[u][v];
        }
        else{
            mean(0)+=octet.filterPos[u][v].u;
            mean(1)+=octet.filterPos[u][v].v;
            mean(2)+=octet.filterPos[u][v].depth;
        }
    }
    gauss.mean = mean*(1.0/double(group.size()));
    Mat33 cov; cov.setZero();
    for (auto it = group.begin(); it!=group.end(); it++){
        if (config.useEuclideanCoordinates){
            cov+=(it->partsPosEucl[u][v]-gauss.mean)*(it->partsPosEucl[u][v]-gauss.mean).transpose();
        }
        else{
            Vec3 pos(it->filterPos[u][v].u, it->filterPos[u][v].v, it->filterPos[u][v].depth);
            cov+=(pos-gauss.mean)*(pos-gauss.mean).transpose();
        }
    }
    gauss.covariance = cov*(1.0/double(group.size()));
}

/// compute Gaussian parameters
void UnbiasedStatsBuilder::computeGaussian(const ViewIndependentPart::Seq& group, GaussianSE3& gauss, unsigned int x, unsigned int y, unsigned int z) const{
    Vec6 mean(Vec6::Zero());
    for (auto it = group.begin(); it!=group.end(); it++){
        Vec6 tmp = gauss.toVector(it->neighbourPoses[x][y][z]);//compute vec6 representation of SE3 transformation
        for (int i=0;i<mean.rows();i++){
            mean(i)+=tmp(i);
        }
    }
    gauss.mean = mean/double(group.size());
    Mat66 cov; cov.setZero();
    for (auto it = group.begin(); it!=group.end(); it++){
        Vec6 pos = gauss.toVector(it->neighbourPoses[x][y][z]);//compute vec6 representation of SE3 transformation
        cov+=(pos-gauss.mean)*(pos-gauss.mean).transpose();
    }
    gauss.covariance = cov/double(group.size());
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(void) {
    builder.reset(new UnbiasedStatsBuilder());
    return builder.get();
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(std::string config) {
    builder.reset(new UnbiasedStatsBuilder(config));
    return builder.get();
}
