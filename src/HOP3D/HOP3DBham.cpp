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
    depthCameraModel.reset(new DepthSensorModel(config.cameraConfig));
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
    compositionConfig = (config.FirstChildElement( "ObjectComposition" )->Attribute( "configFilename" ));
    cameraConfig = (config.FirstChildElement( "CameraModel" )->Attribute( "configFilename" ));
}

/// learining from the dataset
void HOP3DBham::learn(){
    std::vector<hop3d::Octet> octets2layer;
    std::default_random_engine generator(time(0));
    std::uniform_int_distribution<int> distribution(0,3); // filters ids distribution
    int filterSize = 7;
    std::normal_distribution<double> distributionUV(0, filterSize/27.0); // filters ids distribution
    std::normal_distribution<double> distributionDepth(0,0.003);
    int octetsNo = 10000;
    octets2layer.resize(octetsNo);
    for (auto& it: octets2layer){
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

    ///3rd layer
    std::uniform_int_distribution<int> distribution3rd(0,49); // filters ids distribution
    std::vector<hop3d::Octet> octets3layer;
    octetsNo = 200;
    octets3layer.resize(octetsNo);
    for (auto& it: octets3layer){
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

    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;

    ///2nd layer
    std::vector<cv::Mat> vecImages;
    hop3d::Reader reader;
    reader.readMultipleImages("../../resources/depthImages",vecImages);
    std::vector<hop3d::Octet> octets;
    int startId = (int)hierarchy.get()->firstLayer.size();
    for (int layerNo=0;layerNo<config.viewDependentLayersNo;layerNo++){
        if (layerNo==0){
            //imageFilterer->computeOctets(vecImages[0],octets);
            octets=octets2layer;
        }
        else if (layerNo==1){
            startId = int(hierarchy.get()->firstLayer.size()+10000);
            //imageFilterer->getOctets(hierarchy.get()->viewDependentLayers[layerNo-1],octets);
            octets=octets3layer;
        }
        std::cout << "Compute statistics for " << octets.size() << " octets (" << layerNo+2 << "-th layer)\n";
        statsBuilder->computeStatistics(octets, layerNo+2, startId, dictionary);
        std::cout << "Dictionary size (" << layerNo+2 << "-th layer): " << dictionary.size() << "\n";
        partSelector->selectParts(dictionary, *hierarchy, layerNo+2);
        std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
        hierarchy.get()->viewDependentLayers[layerNo]=dictionary;
    }
    notify(*hierarchy);

    //represent all images in parts from 3rd layer
    //imageFilterer->computeImages3rdLayer();
    Dataset dataset(1); dataset.categories[0].objects.resize(1); dataset.categories[0].objects[0].imagesNo=1;
    for (size_t categoryNo=0;categoryNo<dataset.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<dataset.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            objects.push_back(createObjectCompositionOctree(config.compositionConfig));
            for (size_t imageNo=0;imageNo<dataset.categories[categoryNo].objects[objectNo].imagesNo;imageNo++){//for each depth image
                Mat34 cameraPose(Mat34::Identity());
                //int layerNo=3;
                std::vector<ViewDependentPart> parts;
                //get octets of the 3rd layers
                //imageFilterer->getParts(hierarchy.get()->viewDependentLayers[1],layerNo, parts, categoryNo, objectNo, imageNo, cameraPose);
                //move octets into 3D space and update octree representation of the object
                objects.back()->update(parts, cameraPose, *depthCameraModel, *hierarchy);
            }
        }
    }
    ///select part for 4th layer
    //partSelector->selectParts(objects, dictionary, *hierarchy,4)

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
