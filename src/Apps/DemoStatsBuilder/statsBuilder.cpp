
#include "StatisticsBuilder/unbiasedStatsBuilder.h"
#include "ImageFilter/depthImageFilter.h"
#include "Data/Graph.h"
#include "Data/Vocabulary.h"
#include "PartSelector/partSelectorMean.h"
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
        std::string filtererConfig(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
        std::string selectorConfig(config.FirstChildElement( "PartSelector" )->Attribute( "configFilename" ));

        hop3d::StatsBuilder *statsBuilder;
        statsBuilder = hop3d::createUnbiasedStatsBuilder(statsConfig);
        std::cout << statsBuilder->getName() << "\n";

        std::vector<hop3d::Octet> octets;
        std::default_random_engine generator(time(0));
        std::uniform_int_distribution<int> distribution(0,3); // filters ids distribution
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
                    hop3d::ImageCoordsDepth coords(double(j*(filterSize-1))-double(filterSize-1)+distributionUV(generator), double(i*(filterSize-1))-double(filterSize-1)+distributionUV(generator), distributionDepth(generator));
                    it.filterPos[i][j]=coords;
                }
            }
        }
        //Octet.
        octets[0].print();

        hop3d::Hierarchy hierarchy("configGlobal.xml");
        hop3d::ImageFilter* imageFilterer = hop3d::createDepthImageFilter(filtererConfig);
        imageFilterer->setFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml","masks_7x7_0_005.xml");
        imageFilterer->getFilters(hierarchy.firstLayer);
        std::cout << hierarchy.firstLayer[0].normal << "\n";
        std::cout << hierarchy.firstLayer[1].normal << "\n";
        std::cout << hierarchy.firstLayer[2].normal << "\n";


        std::vector<cv::Mat> vecImages;
        hop3d::Reader reader;
        reader.readMultipleImages("../../resources/depthImages",vecImages);
        imageFilterer->computeOctets(vecImages[0],octets);

        hop3d::ViewDependentPart::Seq dictionary;
        statsBuilder->computeStatistics(octets, hierarchy.firstLayer, dictionary);
        std::cout << "groups size: " << dictionary.size() << "\n";
        dictionary[0].print();

        hop3d::PartSelector* partSelector = hop3d::createPartSelectorMean(selectorConfig);
        partSelector->selectParts(dictionary, hierarchy);
        std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
        hierarchy.viewDependentLayers[0]=dictionary;

        std::cout << "Finished\n";
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

