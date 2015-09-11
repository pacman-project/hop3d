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
    ViewDependentPart part;
    //part.
    std::vector<Octet::Seq> groups;
    for (auto it=octets.begin();it!=octets.end();it++){
        if(!isOctetInGroups(*it,groups)){

        }
    }
}

/// is octet in vector
bool UnbiasedStatsBuilder::isOctetInGroups(const Octet& octet, std::vector<Octet::Seq>& groups) const{
    //for (auto it = groups.begin(); groups.end(); it++){
        //if (distance(octet, *it)){
        //    std::cout << "fd\n";
        //}
    //}
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
