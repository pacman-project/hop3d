
#include "StatisticsBuilder/unbiasedStatsBuilder.h"
#include "Data/Vocabulary.h"
#include <iostream>
#include <random>

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

        std::vector<Octet> octets;
        std::default_random_engine generator(time(0));
        std::uniform_int_distribution<int> distribution(0,3); // filters ids distribution
        int filterSize = 7;
        std::normal_distribution<double> distributionUV(filterSize/2.0, filterSize/2.0); // filters ids distribution
        std::normal_distribution<double> distributionDepth(1.0,0.1);
        int octetsNo = 100;
        octets.resize(octetsNo);
        for (auto& it: octets){
            //randomly select filter ids
            for (size_t i=0;i<it.filterIds.size();i++){
                for (size_t j=0;j<it.filterIds[i].size();j++){
                    it.filterIds[i][j]=distribution(generator);
                }
            }
            for (size_t i=0;i<it.filterPos.size();i++){
                for (size_t j=0;j<it.filterPos[i].size();j++){
                    ImageCoordsDepth coords(double(j*(filterSize-1))-double(filterSize-1)+distributionUV(generator), double(i*(filterSize-1))-double(filterSize-1)+distributionUV(generator), distributionDepth(generator));
                    it.filterPos[i][j]=coords;
                }
            }
        }
        //Octet.
        octets[0].print();


        ViewDependentPart::Seq dictionary;
        statsBuilder->computeStatistics(octets, dictionary);
        dictionary[0].print();

        std::cout << "Finished\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

