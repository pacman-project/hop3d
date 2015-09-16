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

    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;
    std::cout << "Compute statistics for " << octets.size() << " octets\n";
    statsBuilder->computeStatistics(octets, hierarchy.get()->firstLayer, dictionary);
    std::cout << "Dictionary size: " << dictionary.size() << "\n";
    //dictionary[0].print();
    partSelector->selectParts(dictionary, *hierarchy);
    std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
    hierarchy.get()->viewDependentLayers[0]=dictionary;
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
