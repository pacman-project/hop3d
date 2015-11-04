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
    depthCameraModel.reset(new DepthSensorModel(config.cameraConfig));

    tinyxml2::XMLDocument configXML;
    std::string filename = "../../resources/" + _config;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID()){
        std::cout << "unable to load global config file.\n";
    }

    if (config.filterType==hop3d::ImageFilter::FILTER_DEPTH){
        imageFilterer = hop3d::createDepthImageFilter(config.filtererConfig);
        imageFilterer->setFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml","masks_7x7_0_005.xml");
    }
    else if (config.filterType==hop3d::ImageFilter::FILTER_NORMAL){
        imageFilterer = hop3d::createNormalImageFilter(config.filtererConfig, config.cameraConfig);
    }
    if (config.datasetType==hop3d::Dataset::DATASET_BORIS){
        dataset = hop3d::createBorisDataset(config.datasetConfig,config.cameraConfig);
    }
    else {// default dataset
        dataset = hop3d::createBorisDataset(config.datasetConfig,config.cameraConfig);
    }
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
    datasetConfig = (config.FirstChildElement( "Dataset" )->Attribute( "configFilename" ));

    config.FirstChildElement( "Filterer" )->QueryIntAttribute("filterType", &filterType);
    config.FirstChildElement( "Dataset" )->QueryIntAttribute("datasetType", &datasetType);
}

/// learining from the dataset
void HOP3DBham::learn(){
    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;

    ///2nd layer
    //std::vector<cv::Mat> vecImages;
    //hop3d::Reader reader;
    //reader.readMultipleImages("../../resources/depthImages",vecImages);
    DatasetInfo datasetInfo;
    dataset->getDatasetInfo(datasetInfo);
    std::vector<hop3d::Octet> octets;
    int startId = (int)hierarchy.get()->firstLayer.size();
    for (int layerNo=0;layerNo<config.viewDependentLayersNo;layerNo++){
        if (layerNo==0){
            for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        cv::Mat image;
                        dataset->getDepthImage((int)categoryNo,(int)objectNo,(int)imageNo,image);
                        std::vector<hop3d::Octet> octetsTmp;
                        imageFilterer->computeOctets(image, (int)categoryNo, (int)objectNo, (int)imageNo, octetsTmp);
                        octets.insert(octets.end(), octetsTmp.begin(), octetsTmp.end());
                    }
                }
            }
        }
        else if (layerNo==1){
            octets.clear();
            startId = int(hierarchy.get()->firstLayer.size()+10000);
            for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        std::vector<hop3d::Octet> octetsTmp;
                        imageFilterer->getOctets((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDependentLayers[layerNo-1], octetsTmp);
                        octets.insert(octets.end(), octetsTmp.begin(), octetsTmp.end());
                    }
                }
            }
        }
        std::cout << "Compute statistics for " << octets.size() << " octets (" << layerNo+2 << "-th layer)\n";
        statsBuilder->computeStatistics(octets, layerNo+2, startId, dictionary);
        std::cout << "Dictionary size (" << layerNo+2 << "-th layer): " << dictionary.size() << "\n";
        partSelector->selectParts(dictionary, *hierarchy, layerNo+2);
        std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
        std::cout << "layerNo " << layerNo << "\n";
        hierarchy.get()->viewDependentLayers[layerNo]=dictionary;
    }

    //represent all images in parts from 3rd layer
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){
            for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                imageFilterer->computeImages3rdLayer((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDependentLayers.back());
            }
        }
    }
    std::vector< std::set<int>> clusters;
    objects.resize(datasetInfo.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            ObjectCompositionOctree object(config.compositionConfig);
            objects[categoryNo].push_back(object);
            std::cout << "object no " << objectNo << "\n";
            for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                //int layerNo=3;
                std::vector<ViewDependentPart> parts;
                //get parts of the 3rd layers
                imageFilterer->getLastVDLayerParts((int)categoryNo, (int)objectNo, (int)imageNo, parts);
                //std::cout << "parts size: " << parts.size() << "\n\n\n\n\n";
                //hierarchy.get()->printIds(parts[0]);
                //hierarchy.get()->printIds(parts[1]);
                //move octets into 3D space and update octree representation of the object
                Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                //std::cout << cameraPose.matrix() << " \n";
                objects[categoryNo][objectNo].update(0, parts, cameraPose, *depthCameraModel, *hierarchy);
            }
            std::vector< std::set<int>> clustersTmp;
            std::cout << "get clusters\n";
            objects[categoryNo][objectNo].getClusters(0,clustersTmp);
            std::cout << " clusters size: " << clustersTmp.size() << "\n";
            std::cout << "finish get clusters\n";
            clusters.reserve( clusters.size() + clustersTmp.size() ); // preallocate memory
            clusters.insert( clusters.end(), clustersTmp.begin(), clustersTmp.end() );
        }
    }
    std::cout << "clusters size: " << clusters.size() << "\n";
    std::vector<ViewIndependentPart> vocabulary;
    std::cout << "create unique clusters:\n";
    partSelector->createUniqueClusters(clusters, vocabulary, *hierarchy);
    std::cout << "4th layer vocabulary size: " << vocabulary.size() << "\n";
    /// First view-independent layer (three and a half layer)
    hierarchy.get()->viewIndependentLayers[0]=vocabulary;
//    for (auto & word : vocabulary){
//        word.print();
//    }
    ///select part for 5th layer
    vocabulary.clear();
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
            std::vector<ViewIndependentPart> vocab;
            objects[categoryNo][objectNo].createNextLayerVocabulary(1, *hierarchy, vocab);
            vocabulary.insert(vocabulary.end(),vocab.begin(), vocab.end());
        }
    }
    //compute statistics
    //select parts
    std::cout << "initial 5th layer vacabulary size: " << vocabulary.size() << "\n";
    std::cout << "Compute statistics for " << vocabulary.size() << " octets (5-th layer)\n";
    /*for (auto & part : vocabulary){
        if (part.id==0)
            part.print();
    }*/
    ViewIndependentPart::Seq newVocabulary;
    statsBuilder->computeStatistics(vocabulary, 5, 0, newVocabulary);
    std::cout << "vocabulary size after statistics: " << newVocabulary.size() << "\n";
    /*for (auto & part : vocabulary){
            part.print();
    }*/
    partSelector->selectParts(newVocabulary, *hierarchy, 5);
    std::cout << "Dictionary size (5-th layer): " << newVocabulary.size() << "\n";
    hierarchy.get()->viewIndependentLayers[1]=newVocabulary;
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (auto & object : objects[categoryNo]){
            object.updateIds(1, hierarchy.get()->viewIndependentLayers[1], *hierarchy);
        }
    }

    ///select part for 6th layer
    vocabulary.clear();
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
            std::vector<ViewIndependentPart> vocab;
            objects[categoryNo][objectNo].createNextLayerVocabulary(2, *hierarchy, vocab);
            vocabulary.insert(vocabulary.end(),vocab.begin(), vocab.end());
        }
    }
    //compute statistics
    //select parts
    std::cout << "initial 6th layer vacabulary size: " << vocabulary.size() << "\n";
    std::cout << "Compute statistics for " << vocabulary.size() << " octets (6-th layer)\n";

    newVocabulary.clear();
    statsBuilder->computeStatistics(vocabulary, 6, 0, newVocabulary);
    std::cout << "vocabulary size after statistics: " << newVocabulary.size() << "\n";

    partSelector->selectParts(newVocabulary, *hierarchy, 6);
    std::cout << "Dictionary size (6-th layer): " << newVocabulary.size() << "\n";
    hierarchy.get()->viewIndependentLayers[2]=newVocabulary;
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (auto & object : objects[categoryNo]){
            object.updateIds(2, hierarchy.get()->viewIndependentLayers[2], *hierarchy);
        }
    }
    //visualization
    notify(*hierarchy);
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (auto & object : objects[categoryNo]){
            std::vector<ViewIndependentPart> objectParts;
            int viewIndLayers = 2;
            for (int i=0;i<viewIndLayers;i++){
                object.getParts(i, objectParts);
                notify(objectParts, i);
            }
        }
    }
    notify3Dmodels();

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
