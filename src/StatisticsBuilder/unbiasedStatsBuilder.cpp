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
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load unbiased stats builder config file.\n";
    tinyxml2::XMLElement * group = config.FirstChildElement( "StatsBuilder" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load statistics builder parameters...\n";
    }
}

/// compute statistics for the set of octets
void UnbiasedStatsBuilder::computeStatistics(const std::vector<Octet>& octets, const Filter::Seq& filters, ViewDependentPart::Seq& dictionary){
    std::vector<Octet::Seq> groups;
    std::vector<Octet::Seq>::iterator groupIter;
    if (config.verbose==1){
        std::cout << "Start ocetes grouping (n=" << octets.size() << ")...\n";
    }
    int iterNo=1;
    for (auto it=octets.begin();it!=octets.end();it++){ // grouping
        if(!isOctetInGroups(*it, filters, groups, groupIter)){
            Octet::Seq group; group.push_back(*it);
            groups.push_back(group);
        }
        else
            (*groupIter).push_back(*it);
        if (config.verbose==1){
            if ((octets.size()>10)&&iterNo%int(octets.size()/10)==0){
                std::cout << "Iteration: " << iterNo << "/" << octets.size() << "\n";
            }
        }
        iterNo++;
    }
    if (config.verbose==1){
        std::cout << "done.\n";
    }
    int partId=1000;
    if (config.verbose==1){
        std::cout << "Compute Gaussians for " << groups.size() << " groups...\n";
    }
    iterNo=1;
    for (auto it = groups.begin(); it!=groups.end(); it++){ //compute statistics
        ViewDependentPart part;
        computeGaussians(*it, part);
        part.partIds=it->back().filterIds;//copy octet
        part.layerId = 2;
        part.id = partId;
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

/// Compute Gaussians for the group of octets
void UnbiasedStatsBuilder::computeGaussians(Octet::Seq& group, ViewDependentPart& part) const{
    for (size_t i = 0; i<part.gaussians.size(); i++){
        for (size_t j = 0; j<part.gaussians[i].size(); j++){
            Gaussian3D gauss;
            computeGaussian(group, gauss, (unsigned int)i, (unsigned int)j);
            part.gaussians[i][j].mean = gauss.mean;
            part.gaussians[i][j].covariance = gauss.covariance;
        }
    }
}

/// compute Gaussian parameters
void UnbiasedStatsBuilder::computeGaussian(const Octet::Seq& group, Gaussian3D& gauss, unsigned int u, unsigned int v) const{
    Vec3 mean(0,0,0);
    for (auto it = group.begin(); it!=group.end(); it++){
        mean(0)+=it->filterPos[u][v].u;
        mean(1)+=it->filterPos[u][v].v;
        mean(2)+=it->filterPos[u][v].depth;
    }
    gauss.mean = mean*(1.0/double(group.size()));
    Mat33 cov; cov.setZero();
    for (auto it = group.begin(); it!=group.end(); it++){
        Vec3 pos(it->filterPos[u][v].u, it->filterPos[u][v].v, it->filterPos[u][v].depth);
        cov+=(pos-gauss.mean)*(pos-gauss.mean).transpose();
    }
    gauss.covariance = cov*(1.0/double(group.size()));
}

/// is octet in vector
bool UnbiasedStatsBuilder::isOctetInGroups(const Octet& octet, const Filter::Seq& filters, std::vector<Octet::Seq>& groups, std::vector<Octet::Seq>::iterator& groupIt) const{
    for (std::vector<Octet::Seq>::iterator it = groups.begin(); it!=groups.end(); it++){
        if (Octet::distance(octet, it->back(), filters)==0){
            groupIt = it;
            return true;
        }
    }
    return false;
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(void) {
    builder.reset(new UnbiasedStatsBuilder());
    return builder.get();
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(std::string config) {
    builder.reset(new UnbiasedStatsBuilder(config));
    return builder.get();
}
