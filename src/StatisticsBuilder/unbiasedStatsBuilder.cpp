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
    //tinyxml2::XMLElement * model = config.FirstChildElement( "Filterer" );
    //model->FirstChildElement( "parameters" )->QueryIntAttribute("filtersNo", &filtersNo);
    std::cout << "Load filter parameters...\n";
    //std::cout << "Filters no.: " << filtersNo << "\n";
}

/// compute statistics for the set of octets
void UnbiasedStatsBuilder::computeStatistics(const std::vector<Octet>& octets, ViewDependentPart::Seq& dictionary){
    std::vector<Octet::Seq> groups;
    std::vector<Octet::Seq>::iterator groupIter;
    for (auto it=octets.begin();it!=octets.end();it++){
        if(!isOctetInGroups(*it,groups, groupIter)){
            Octet::Seq group; group.push_back(*it);
            groups.push_back(group);
        }
        else
            (*groupIter).push_back(*it);
    }
    int partId=1000;
    for (auto it = groups.begin(); it!=groups.end(); it++){
        ViewDependentPart part;
        computeGaussians(*it, part);
        part.id = partId;
        partId++;
        dictionary.push_back(part);
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
        cov+=(pos-mean)*(pos-mean).transpose();
    }
    gauss.covariance = cov*(1.0/double(group.size()));
}

/// is octet in vector
bool UnbiasedStatsBuilder::isOctetInGroups(const Octet& octet, std::vector<Octet::Seq>& groups, std::vector<Octet::Seq>::iterator& groupIt) const{
    for (std::vector<Octet::Seq>::iterator it = groups.begin(); it!=groups.end(); it++){
        if (distance(octet, it->back())==0){
            groupIt = it;
            return true;
        }
    }
    return false;
}

/// Compute distance between octets
double UnbiasedStatsBuilder::distance(const Octet& octetA, const Octet& octetB) const{
    if (octetA.filterIds==octetB.filterIds)
        return 0;
    else
        return 1.0; //!!!compute distance here
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(void) {
    builder.reset(new UnbiasedStatsBuilder());
    return builder.get();
}

hop3d::StatsBuilder* hop3d::createUnbiasedStatsBuilder(std::string config) {
    builder.reset(new UnbiasedStatsBuilder(config));
    return builder.get();
}
