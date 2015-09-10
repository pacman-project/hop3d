#include <iostream>
#include "StatisticsBuilder/unbiasedStatsBuilder.h"
#include "Data/Vocabulary.h"

int main(void){
    try {
        tinyxml2::XMLDocument config;
        config.LoadFile("../../resources/configGlobal.xml");
        if (config.ErrorID()){
            std::cout << "unable to load global config file.\n";
            return 1;
        }
        std::string statsConfig(config.FirstChildElement( "StatisticsBuilder" )->Attribute( "configFilename" ));

        StatsBuilder *statsBuilder;
        statsBuilder = createUnbiasedStatsBuilder(statsConfig);
        std::cout << statsBuilder->getName() << "\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

