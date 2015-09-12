
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

        hop3d::StatsBuilder *statsBuilder;
        statsBuilder = hop3d::createUnbiasedStatsBuilder(statsConfig);
        std::cout << statsBuilder->getName() << "\n";

        std::vector<hop3d::Octet> octets;
        std::default_random_engine generator(time(0));
        std::uniform_int_distribution<int> distribution(0,0); // filters ids distribution
        int filterSize = 7;
        std::normal_distribution<double> distributionUV(filterSize/2.0, filterSize/2.0); // filters ids distribution
        std::normal_distribution<double> distributionDepth(1.0,0.1);
        int octetsNo = 10000;
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
<<<<<<< HEAD
                    ImageCoordsDepth coords(double(j*(filterSize-1))-double(filterSize-1)+distributionUV(generator), double(i*(filterSize-1))-double(filterSize-1)+distributionUV(generator), distributionDepth(generator));
                    if (i==0 && j==0) std::cout << coords.u << ", ";
=======
                    hop3d::ImageCoordsDepth coords(double(j*(filterSize-1))-double(filterSize-1)+distributionUV(generator), double(i*(filterSize-1))-double(filterSize-1)+distributionUV(generator), distributionDepth(generator));
>>>>>>> 201d9a8ee5d6b7ab1960364d9b6b309b770e6f51
                    it.filterPos[i][j]=coords;
                }
            }
        }
        //Octet.
        octets[0].print();


<<<<<<< HEAD
        ViewDependentPart::Seq dictionary;
        std::cout << "compute statistics\n";
=======
        hop3d::ViewDependentPart::Seq dictionary;
>>>>>>> 201d9a8ee5d6b7ab1960364d9b6b309b770e6f51
        statsBuilder->computeStatistics(octets, dictionary);
        std::cout << "groups size: " << dictionary.size() << "\n";
        dictionary[0].print();

        std::cout << "Finished\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

