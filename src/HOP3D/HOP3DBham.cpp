#include "HOP3D/HOP3DBham.h"

using namespace hop3d;

/// A single instance of unbiased stats builder
HOP3DBham::Ptr hop3dBham;

HOP3DBham::HOP3DBham(void) : HOP3D("Unbiased Statistics Builder", HOP3D_BHAM) {
}

/// Construction
HOP3DBham::HOP3DBham(std::string _config) :
        HOP3D("Unbiased Statistics Builder", HOP3D_BHAM), config(_config) {
    statsBuilder = hop3d::createUnbiasedStatsBuilder(config.statsConfig);
    hierarchy.reset(new Hierarchy(_config));
    partSelector = hop3d::createPartSelectorMean(config.selectorConfig);
    imageFilterer = hop3d::createDepthImageFilter(config.filtererConfig);
    imageFilterer->setFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml","masks_7x7_0_005.xml");
}

#ifdef QVisualizerBuild
///Attach visualizer
void HOP3DBham::attachVisualizer(QGLVisualizer* visualizer){
    hop3dBham->attach(visualizer);
}
#endif

/// Destruction
HOP3DBham::~HOP3DBham(void) {
}

///config class constructor
HOP3DBham::Config::Config(std::string configFilename){
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        std::cout << "unable to load unbiased stats builder config file.\n";
    tinyxml2::XMLElement * group = config.FirstChildElement( "Hierarchy" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load HOP3DBham parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewDependentLayersNo", &viewDependentLayersNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewIndependentLayersNo", &viewIndependentLayersNo);

    statsConfig = (config.FirstChildElement( "StatisticsBuilder" )->Attribute( "configFilename" ));
    selectorConfig = (config.FirstChildElement( "PartSelector" )->Attribute( "configFilename" ));
    filtererConfig = (config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
}

/// learining from the dataset
void HOP3DBham::learn(){
    std::vector<hop3d::Octet> octets;
    std::default_random_engine generator(time(0));
    std::uniform_int_distribution<int> distribution(0,3); // filters ids distribution
    int filterSize = 7;
    std::normal_distribution<double> distributionUV(0, filterSize/27.0); // filters ids distribution
    std::normal_distribution<double> distributionDepth(0,0.003);
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
                if ((i==(it.filterPos.size()/2))&&(j==(it.filterPos.size()/2))){
                    coords.u=0; coords.v=0; coords.depth=0;
                }
                it.filterPos[i][j]=coords;
            }
        }
    }

    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;

    ///2nd layer
    std::vector<cv::Mat> vecImages;
    hop3d::Reader reader;
    reader.readMultipleImages("../../resources/depthImages",vecImages);
    //imageFilterer->computeOctets(vecImages[0],octets);
    std::cout << "Compute statistics for " << octets.size() << " octets (2nd layer)\n";
    int startId = (int)hierarchy.get()->firstLayer.size();
    statsBuilder->computeStatistics(octets, 2, startId, dictionary);
    std::cout << "Dictionary size (2nd layer): " << dictionary.size() << "\n";
    partSelector->selectParts(dictionary, *hierarchy, 2);
    std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
    hierarchy.get()->viewDependentLayers[0]=dictionary;

    ///3rd layer
    std::uniform_int_distribution<int> distribution3rd(0,49); // filters ids distribution
    octetsNo = 200;
    octets.clear();
    octets.resize(octetsNo);
    for (auto& it: octets){
        //randomly select filter ids
        for (size_t i=0;i<it.filterIds.size();i++){
            for (size_t j=0;j<it.filterIds[i].size();j++){
                it.filterIds[i][j]=distribution3rd(generator);
            }
        }
        for (size_t i=0;i<it.filterPos.size();i++){
            for (size_t j=0;j<it.filterPos[i].size();j++){
                hop3d::ImageCoordsDepth coords(double(j*(3*filterSize-1))-double(3*filterSize-1)+distributionUV(generator), double(i*(3*filterSize-1))-double(3*filterSize-1)+distributionUV(generator), distributionDepth(generator));
                if ((i==(it.filterPos.size()/2))&&(j==(it.filterPos.size()/2))){
                    coords.u=0; coords.v=0; coords.depth=0;
                }
                it.filterPos[i][j]=coords;
            }
        }
    }
    //imageFilterer->getOctets(dictionary,octates);
    std::cout << "Compute statistics for " << octets.size() << " octets (3rd layer)\n";
    startId = int(hierarchy.get()->firstLayer.size()+10000);
    dictionary.clear();
    statsBuilder->computeStatistics(octets, 3, startId, dictionary);
    std::cout << "Dictionary size (3rd layer): " << dictionary.size() << "\n";
    partSelector->selectParts(dictionary, *hierarchy, 3);
    std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
    hierarchy.get()->viewDependentLayers[1]=dictionary;

    notify(*hierarchy);
    //dictionary[0].print();

    std::cout << "Finished\n";
}

hop3d::HOP3D* hop3d::createHOP3DBham(void) {
    hop3dBham.reset(new HOP3DBham());
    return hop3dBham.get();
}

hop3d::HOP3D* hop3d::createHOP3DBham(std::string config) {
    hop3dBham.reset(new HOP3DBham(config));
    return hop3dBham.get();
}
