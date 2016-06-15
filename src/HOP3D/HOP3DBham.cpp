#include "hop3d/HOP3D/HOP3DBham.h"
#include <random>
#include <thread>
#include <chrono>

using namespace hop3d;

/// A single instance of unbiased stats builder
HOP3DBham::Ptr hop3dBham;

HOP3DBham::HOP3DBham(void) : HOP3D("Unbiased Statistics Builder", HOP3D_BHAM){
}

/// Construction
HOP3DBham::HOP3DBham(std::string _config) :
        HOP3D("Unbiased Statistics Builder", HOP3D_BHAM), config(_config) {
    statsBuilder = hop3d::createUnbiasedStatsBuilder(config.statsConfig);
    hierarchy.reset(new Hierarchy(_config));
    if (config.partSelectorType==hop3d::PartSelector::SELECTOR_MEAN)
        partSelector = hop3d::createPartSelectorMean(config.selectorConfig);
    else if (config.partSelectorType==hop3d::PartSelector::SELECTOR_AGGLOMERATIVE)
        partSelector = hop3d::createPartSelectorAgglomerative(config.selectorConfig);
    depthCameraModel.reset(new DepthSensorModel(config.cameraConfig));

    tinyxml2::XMLDocument configXML;
    std::string filename = _config;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID())
		throw std::runtime_error("unable to load global config file: " + filename);

    if (config.verbose == 1) {
        std::cout << "Create depth image filterer...\n";
    }
    if (config.filterType==hop3d::ImageFilter::FILTER_DEPTH){
        imageFilterer = hop3d::createDepthImageFilter(config.filtererConfig);
        imageFilterer->setFilters("filters_7x7_0_005.xml","normals_7x7_0_005.xml","masks_7x7_0_005.xml");
    }
    else if (config.filterType==hop3d::ImageFilter::FILTER_NORMAL){
        imageFilterer = hop3d::createNormalImageFilter(config.filtererConfig, config.cameraConfig);
    }
    if (config.verbose == 1) {
        std::cout << "Done\n";
        std::cout << "Create train and inference dataset wrapper...\n";
    }
    if (config.datasetType==hop3d::Dataset::DATASET_BORIS){
        datasetTrain = hop3d::createBorisDataset(config.datasetConfig,config.cameraConfig);
        datasetTest = hop3d::createBorisDataset(config.configFilenameTest,config.cameraConfig);
        datasetIncremental = hop3d::createBorisDataset(config.datasetConfigIncremental,config.cameraConfig);
    }
    else if (config.datasetType==hop3d::Dataset::DATASET_PACMAN){
        datasetTrain = hop3d::createPacmanDataset(config.datasetConfig,config.cameraConfig);
        datasetTest = hop3d::createPacmanDataset(config.configFilenameTest,config.cameraConfig);
    }
    else {// default dataset
        datasetTrain = hop3d::createBorisDataset(config.datasetConfig,config.cameraConfig);
        datasetTest = hop3d::createBorisDataset(config.configFilenameTest,config.cameraConfig);
        datasetIncremental = hop3d::createBorisDataset(config.datasetConfigIncremental,config.cameraConfig);
    }
    if (config.verbose == 1) {
        std::cout << "Done\n";
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
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
        throw std::runtime_error("unable to load hop3d config file: " + filename);
    tinyxml2::XMLElement * group = config.FirstChildElement( "Hierarchy" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load HOP3DBham parameters...\n";
    }
    size_t found = configFilename.find_last_of("/\\");
    std::string prefix = configFilename.substr(0,found+1);

    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewDependentLayersNo", &viewDependentLayersNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewIndependentLayersNo", &viewIndependentLayersNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("inferenceUpToLayer", &inferenceUpToLayer);

    group->FirstChildElement( "save2file" )->QueryBoolAttribute("save2file", &save2file);
    filename2save = group->FirstChildElement( "save2file" )->Attribute( "filename2save" );
    group->FirstChildElement( "save2file" )->QueryBoolAttribute("saveInference", &saveInference);
    filename2saveInference = group->FirstChildElement( "save2file" )->Attribute( "filename2saveInference" );

    statsConfig = prefix+(config.FirstChildElement( "StatisticsBuilder" )->Attribute( "configFilename" ));
    config.FirstChildElement( "PartSelector" )->QueryIntAttribute("selectorType", &partSelectorType);
    selectorConfig = prefix+(config.FirstChildElement( "PartSelector" )->Attribute( "configFilename" ));
    filtererConfig = prefix+(config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
    compositionConfig = prefix+(config.FirstChildElement( "ObjectComposition" )->Attribute( "configFilename" ));
    cameraConfig = prefix+(config.FirstChildElement( "CameraModel" )->Attribute( "configFilename" ));
    datasetConfig = prefix+(config.FirstChildElement( "Dataset" )->Attribute( "configFilename" ));
    configFilenameTest = prefix+(config.FirstChildElement( "Dataset" )->Attribute( "configFilenameTest" ));
    datasetConfigIncremental = prefix+(config.FirstChildElement( "Dataset" )->Attribute( "configFilenameIncremental" ));

    config.FirstChildElement( "Filterer" )->QueryIntAttribute("filterType", &filterType);
    config.FirstChildElement( "Dataset" )->QueryIntAttribute("datasetType", &datasetType);
    config.FirstChildElement( "QVisualizer" )->QueryBoolAttribute("useVisualization", &useVisualization);
}

/// get set of ids from hierarchy for the given input point
/*void HOP3DBham::getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const{
    int categoryNo, objectNo, imageNo = 0;
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getPartsIds(categoryNo, objectNo, imageNo, u, v, ids);
}*/

/// get training dataset info
void HOP3DBham::getDatasetInfo(hop3d::DatasetInfo& _dataset, bool inference) const{
    if (inference)
        datasetTest->getDatasetInfo(_dataset);
    else
        datasetTrain->getDatasetInfo(_dataset);
}

/// returns paths for each cloud/image
void HOP3DBham::getCloudPaths(std::vector<std::string>& paths, bool inference) const{
    paths.clear();
    hop3d::DatasetInfo info;
    getDatasetInfo(info,inference);
    for (const auto &category : info.categories)
        for (const auto &object : category.objects)
            for (const auto &path : object.fullPaths)
                paths.push_back(path);
}

/// get cloud from dataset
void HOP3DBham::getCloud(const cv::Mat& depthImage, hop3d::PointCloud& cloud) const{
    hop3d::PointCloudUV cloudUV;
    imageFilterer->getCloud(depthImage, cloudUV);
    cloud.clear();
    cloud.reserve(cloudUV.size());
    for (auto &point : cloudUV){
        hop3d::PointNormal p3d;
        p3d=point;
        cloud.push_back(p3d);
    }
}

/// get cloud from dataset
void HOP3DBham::getCloud(const std::string& path, hop3d::PointCloud& cloud, bool inference) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    cv::Mat depthImage;
    if (inference){
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            depthImage = infDataIt->second.depthImage;
        }
        else{
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            datasetTest->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
        }
        getCloud(depthImage, cloud);
    }
    else {
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        datasetTrain->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
        getCloud(depthImage, cloud);
    }
}

/// get number of points in the point cloud
//size_t HOP3DBham::getNumOfPoints(int categoryNo, int objectNo, int imageNo, bool inference) const {
//    if (inference)
//        return datasetTest->getNumOfPoints(categoryNo, objectNo, imageNo);
//    else
//        return datasetTrain->getNumOfPoints(categoryNo, objectNo, imageNo);
//}

/// get point from the point cloud
/*void HOP3DBham::getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const{
    dataset->getPoint(categoryNo, objectNo, imageNo, pointNo, point);
}*/

/// get camera pose
//void HOP3DBham::getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose, bool inference) const{
//    if (inference)
//        cameraPose = datasetTest->getCameraPose(categoryNo, objectNo, imageNo);
//    else
//        cameraPose = datasetTrain->getCameraPose(categoryNo, objectNo, imageNo);
//}

/// get camera pose
void HOP3DBham::getSensorFrame(const std::string& path, Mat34& cameraPose, bool inference) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    if (inference) {
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            cameraPose = infDataIt->second.cameraPose;
        }
        else{
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            cameraPose = datasetTest->getCameraPose(categoryNo, objectNo, imageNo);
        }
    }
    else{
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        cameraPose = datasetTrain->getCameraPose(categoryNo, objectNo, imageNo);
    }
}

/// get hierarchy graph
void HOP3DBham::getHierarchy(Hierarchy::IndexSeqMap& hierarchyGraph) const{
    hierarchy.get()->computeGraph(hierarchyGraph);
}

/// get parts realization
void HOP3DBham::getPartsRealisation(int categoryNo, int objectNo, int imageNo, const Mat34& cameraPose, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const{
    parts.clear();
    int idIncrement=10000;
    for (int layerNo = 0;layerNo<(int)hierarchy.get()->viewDependentLayers.size()+1;layerNo++){
        std::vector<ViewDependentPart> partsView;
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<ViewDependentPart> partsViewTmp;
            imageFilterer->getPartsRealisation(overlapNo, categoryNo, objectNo, imageNo,layerNo,partsViewTmp, inference);
            partsView.insert(partsView.end(), partsViewTmp.begin(), partsViewTmp.end());
        }
        for (auto& part : partsView){
            ViewIndependentPart::Part3D part3D;
            part3D.id = ((layerNo)*idIncrement)+part.id;
            //Vec3 point3d;
            //depthCameraModel.get()->getPoint(part.location.u, part.location.v, part.location.depth, point3d);
            Mat34 pointPose(Mat34::Identity());
            pointPose.translation() = part.locationEucl;
            part3D.pose = cameraPose * pointPose*part.offset;
            part3D.realisationId = part.realisationId;
            parts.push_back(part3D);
        }
    }
    /// volumetric layers
    for (int layerNo = 0;layerNo<(int)hierarchy.get()->viewIndependentLayers.size();layerNo++){
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<ViewIndependentPart::Part3D> partsViewTmp;
            if (inference)
                objectsInference[categoryNo][objectNo].getPartsRealisation(layerNo, overlapNo, partsViewTmp);
            else
                objects[categoryNo][objectNo].getPartsRealisation(layerNo, overlapNo, partsViewTmp);
            for (auto& part : partsViewTmp){
                part.id += ((int)hierarchy.get()->viewDependentLayers.size()+1+layerNo)*idIncrement;
            }
            parts.insert(parts.end(), partsViewTmp.begin(), partsViewTmp.end());
        }
    }
}

/// get parts realization
void HOP3DBham::getPartsRealisationCloud(int categoryNo, int objectNo, int imageNo, const cv::Mat& depthImage, const Mat34& cameraPose, PartsClouds& parts, bool inference) const{
    parts.clear();
    Hierarchy::IndexSeqMap points2parts;
    getCloud2PartsMap(categoryNo, objectNo, imageNo, depthImage, cameraPose, points2parts, inference);
    convertPartsMap2PartsCloud(points2parts, parts);
    /*for (auto &element : parts){
        std::cout << "part id: " << element.first << " points: ";
        for (auto & pointId : element.second){
            std::cout << pointId << ", ";
        }
        std::cout << "\n";
    }*/
}

/// get realisations graph
void HOP3DBham::getRealisationsGraph(const std::string& path, Hierarchy::IndexSetMap& realisationsGraph, bool inference) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    Mat34 cameraPose;
    cv::Mat depthImage;
    if (inference) {
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            cameraPose = infDataIt->second.cameraPose;
            depthImage = infDataIt->second.depthImage;
        }
        else{
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            cameraPose = datasetTest->getCameraPose(categoryNo, objectNo, imageNo);
            datasetTest->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
        }
    }
    else{
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        cameraPose = datasetTrain->getCameraPose(categoryNo, objectNo, imageNo);
        datasetTrain->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    }
    getRealisationsGraph(categoryNo, objectNo, imageNo, depthImage, cameraPose, realisationsGraph, inference);
}

/// get realisations graph
void HOP3DBham::getRealisationsGraph(int categoryNo, int objectNo, int imageNo, const cv::Mat& depthImage, const Mat34& cameraPose, Hierarchy::IndexSetMap& realisationsGraph, bool inference) const{
    PointCloudUV cloud;
    realisationsGraph.clear();
    imageFilterer->getCloud(depthImage, cloud);
    for (auto &point : cloud){
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<int> ids;
            getRealisationsIds(overlapNo, categoryNo, objectNo, imageNo, cameraPose, point.u,point.v,point.position(2),ids, inference);
            int idPrev=-1;
            int idNo=0;
            for (auto & partId : ids){
                //std::cout << "partId " << partId << ", ";
                if (idNo==0&&partId>-1){
                    realisationsGraph.insert(std::make_pair(partId,std::set<unsigned int>()));
                }
                else if (idPrev>-1&&partId>-1){// id_{layer}->ids_{layer-1}
                    realisationsGraph[partId].insert(idPrev);
                }
                if (idNo==2)//not parametrized
                    idPrev = ids[1];
                else
                    idPrev=partId;
                idNo++;
            }
        }
        //std::cout << "\n"; getchar();
    }
    /*for (auto &element : realisationsGraph){
        std::cout << "part id: " << element.first << " is build from parts: ";
        for (auto & partId : element.second){
            std::cout << partId << ",";
        }
        std::cout << "\n";
    }*/
}

/// get parts realisation
void HOP3DBham::getPartsRealisationCloud(const std::string& path, PartsClouds& parts, bool inference) const{
    parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    cv::Mat depthImage;
    Mat34 cameraPose;
    if (inference) {
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            depthImage = infDataIt->second.depthImage;
            cameraPose = infDataIt->second.cameraPose;
        }
        else{
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            datasetTest->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
            cameraPose = datasetTest->getCameraPose(categoryNo, objectNo, imageNo);
        }
    }
    else{
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        datasetTrain->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
        cameraPose = datasetTrain->getCameraPose(categoryNo, objectNo, imageNo);
    }
    getPartsRealisationCloud(categoryNo, objectNo, imageNo, depthImage, cameraPose, parts, inference);
}

/// get maps from point to part realisation
void HOP3DBham::getCloud2PartsMap(int categoryNo, int objectNo, int imageNo, const cv::Mat& depthImage, const Mat34& cameraPose, Hierarchy::IndexSeqMap& points2parts, bool inference) const{
    points2parts.clear();
    PointCloudUV cloud;
    imageFilterer->getCloud(depthImage, cloud);
    std::uint32_t pointIdx=0;
    for (auto &point : cloud){
        std::vector<std::uint32_t> idsParts;
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<int> ids;
            getRealisationsIds(overlapNo, categoryNo,objectNo,imageNo, cameraPose, point.u,point.v, point.position(2),ids, inference);
            for (size_t layerNo=0;layerNo<ids.size();layerNo++){
                if (ids[layerNo]>=0){
                    idsParts.push_back((std::uint32_t)ids[layerNo]);
                    //idsParts.push_back((((std::uint32_t)layerNo)*10000)+(std::uint32_t)ids[layerNo]);//for model ids
                    //std::cout << "point " << pointIdx << " -> " << (std::uint32_t)ids[layerNo] << " lay no " << layerNo << "\n";
                }
            }
        }
        points2parts.insert(std::make_pair(pointIdx,idsParts));
        pointIdx++;
    }
}

/// get maps from point to part realisation
void HOP3DBham::getCloud2PartsMap(const std::string& path, Hierarchy::IndexSeqMap& points2parts, bool inference) const{
    points2parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    cv::Mat depthImage;
    Mat34 cameraPose;
    if (inference) {
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            depthImage = infDataIt->second.depthImage;
            cameraPose = infDataIt->second.cameraPose;
        }
        else{
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            datasetTest->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
            cameraPose = datasetTest->getCameraPose(categoryNo, objectNo, imageNo);
        }
    }
    else{
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        datasetTrain->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
        cameraPose = datasetTrain->getCameraPose(categoryNo, objectNo, imageNo);
    }
    getCloud2PartsMap(categoryNo, objectNo, imageNo, depthImage, cameraPose, points2parts, inference);
}

/// get parts realization
void HOP3DBham::getPartsRealisation(const std::string& path, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const{
    parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    Mat34 cameraPose;
    if (inference){
        std::map<std::string, InferenceObject>::const_iterator infDataIt;
        if (!inferenceData.find(path,categoryNo, objectNo, imageNo, infDataIt)){
            datasetTest->translateString(path, categoryNo, objectNo, imageNo);
            cameraPose = datasetTest->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo);
        }
        else {
            cameraPose = infDataIt->second.cameraPose;
        }
    }
    else{
        datasetTrain->translateString(path, categoryNo, objectNo, imageNo);
        cameraPose = datasetTrain->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo);
    }
    getPartsRealisation(categoryNo, objectNo, imageNo, cameraPose, parts, inference);
}

/// learining from the dataset
void HOP3DBham::learn(){
    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;
    datasetTrain->getDatasetInfo(datasetInfoTrain);
    std::vector<hop3d::Octet> octets;
    int startId = (int)hierarchy.get()->firstLayer.size();
    for (int layerNo=0;layerNo<config.viewDependentLayersNo;layerNo++){
        if (layerNo==0){
            for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTrain.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        cv::Mat image;
                        datasetTrain->getDepthImage((int)categoryNo,(int)objectNo,(int)imageNo,image);
                        std::vector<hop3d::Octet> octetsTmp;
                        imageFilterer->computeOctets(image, (int)categoryNo, (int)objectNo, (int)imageNo, octetsTmp, false);
                        octets.insert(octets.end(), octetsTmp.begin(), octetsTmp.end());
                    }
                }
            }
        }
        else if (layerNo==1){
            octets.clear();
            startId = int(hierarchy.get()->firstLayer.size()+10000);
            for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTrain.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        std::vector<hop3d::Octet> octetsTmp;
                        imageFilterer->getOctets((int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, octetsTmp, false);
                        octets.insert(octets.end(), octetsTmp.begin(), octetsTmp.end());
                    }
                }
            }
        }
        std::cout << "Compute statistics for " << octets.size() << " octets (" << layerNo+2 << " layer)\n";
        //statsBuilder->computeStatistics(octets, layerNo+2, startId, dictionary);
        statsBuilder->vocabularyFromOctets(octets, layerNo+2, startId, dictionary);
        std::cout << "Dictionary size (" << layerNo+2 << " layer): " << dictionary.size() << "\n";
        partSelector->selectParts(dictionary, *hierarchy, layerNo+2);
        std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
        hierarchy.get()->viewDependentLayers[layerNo]=dictionary;
    }
    //represent/explain all images in parts from i-th layer
    for (size_t layerNo=0; layerNo< hierarchy.get()->viewDependentLayers.size();layerNo++){
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTrain.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        //std::cout << layerNo << " " << overlapNo << " " << categoryNo << " " << objectNo << " " << imageNo <<"\n";
                        imageFilterer->computePartsImage(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, (int)layerNo, false);
                        //std::cout << "end\n";
                    }
                }
            }
        }
    }
    objects.resize(datasetInfoTrain.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            ObjectCompositionOctree object(config.compositionConfig);
            objects[categoryNo].push_back(object);
            for (size_t imageNo=0;imageNo<datasetInfoTrain.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                //int layerNo=3;
                std::vector<ViewDependentPart> parts;
                //get parts of the 3rd layers
                imageFilterer->getLayerParts((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDepPartsFromLayerNo-1, parts, false);
                //move octets into 3D space and update octree representation of the object
                Mat34 cameraPose(datasetTrain->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                objects[categoryNo][objectNo].updatePCLGrid(parts, cameraPose);
            }
        }
    }
    std::vector<ViewIndependentPart> vocabulary;
    ObjectCompositionOctree::setRealisationCounter(imageFilterer->getRealisationsNo()+10000);
    for (size_t layerNo=0;layerNo<(size_t)config.viewIndependentLayersNo;layerNo++){
        vocabulary.clear();
        for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> voc;
                objects[categoryNo][objectNo].createNextLayerVocabulary((int)layerNo, voc);
                vocabulary.insert( vocabulary.end(), voc.begin(), voc.end() );
            }
        }
        /*for (auto &word : vocabulary){
            word.print();
        }
        getchar();*/
        std::cout << layerNo+4 << " layer init vocabulary size: " << vocabulary.size() << "\n";
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        partSelector->selectParts(vocabulary, *hierarchy, int(layerNo+4));
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        std::cout << "Clusterization took = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
        std::cout << "Dictionary size (" << layerNo+4 << "-th layer): " << vocabulary.size() << "\n";
        /// First view-independent layer (three and a half layer)
        hierarchy.get()->viewIndependentLayers[layerNo]=vocabulary;
        for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
                objects[categoryNo][objectNo].updateVoxelsPose((int)layerNo, vocabulary, *hierarchy);
            }
        }
        //std::cout << "data explained\n";
    }
    // save hierarchy to file
    if(config.save2file){
        std::cout << "save 2 file\n";
        std::ofstream ofsHierarchy(config.filename2save);
        ofsHierarchy << *hierarchy;
        for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
                ofsHierarchy << objects[categoryNo][objectNo];
            }
        }
        ((NormalImageFilter*)imageFilterer)->save2file(ofsHierarchy, false);
        ofsHierarchy.close();
        std::cout << "saved\n";
    }
    //visualization
    notifyVisualizer();
    /*Hierarchy::IndexSeqMap hierarchyGraph;
    getHierarchy(hierarchyGraph);
    std::vector<ViewIndependentPart::Part3D> parts;
    getPartsRealisation(0,0,0, parts);
    for (auto &part : parts){
        std::cout << "part id " << part.id << "\n";
        std::cout << "part realisation id " << part.realisationId << "\n";
        std::cout << "part pose\n" << part.pose.matrix() << "\n";
    }
    Hierarchy::IndexSeqMap points2parts;
    getCloud2PartsMap(0,0,0, points2parts);
    PartsClouds partsCloud;
    getPartsRealisationCloud(0,0,0,partsCloud);
    Hierarchy::IndexSetMap realisationsGraph;
    getRealisationsGraph(0,0,0, realisationsGraph);
    for (auto &part : parts){
        std::cout << "part id " << part.id << "\n";
        std::cout << "part realisation id " << part.realisationId << "\n";
        std::cout << "is made from parts: ";
        for (auto &partId : realisationsGraph[part.realisationId])
            std::cout << partId << ", ";
        std::cout << "\n";
    }*/
    /*std::cout << "octets size: " << octets2nd.size() << "\n";
    std::cout << "vocabulary size: " << hierarchy.get()->viewDependentLayers[0].size() << "\n";
    for (auto& octet : octets2nd){
        int partNo=0;
        for (auto& part : hierarchy.get()->viewDependentLayers[0]){
            if (part.partIds==octet.partIds){
                std::cout << "part Id: " << partNo << "\n";
                break;
            }
            partNo++;
        }
        std::vector<int> ids;
        getPartsIds(0,0,0, (int)octet.filterPos[1][1].v, (int)octet.filterPos[1][1].u, ids);
        std::cout << "readed Id: " << ids[1] << "\n";
        if (ids[1]<0)
            hierarchy.get()->viewDependentLayers[0][partNo].print();
    }*/
    //std::vector<int> ids;
    //getPartsIds(0,0,1, 339, 292, ids);
    //getPartsIds(0,0,1, 350, 290, ids);
    //getPartsIds(0,0,1, 333, 292, ids);
    hierarchy->showInfo();
    std::cout << "Finished\n";
}

/// learning from the dataset
void HOP3DBham::learnIncremental(void){
    if (hierarchy.get()->viewDependentLayers[0].size()==0)
        throw std::runtime_error("Train or load hierarchy first\n");
    datasetIncremental->getDatasetInfo(datasetInfoIncremental);
    std::cout << "Start incremental learning\n";
    for (int layerNo=0;layerNo<config.viewDependentLayersNo;layerNo++){
        for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){
            for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){
                for (size_t imageNo=0;imageNo<datasetInfoIncremental.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                    cv::Mat image;
                    datasetIncremental->getDepthImage((int)categoryNo,(int)objectNo,(int)imageNo,image);
                    std::vector<hop3d::Octet> octetsTmp;
                    if (layerNo==0)
                        imageFilterer->computeOctets(image, (int)categoryNo, (int)objectNo, (int)imageNo, octetsTmp, true);
                    else if (layerNo==1)
                        imageFilterer->getOctets((int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, octetsTmp, true);
                }
            }
        }
        std::vector<ViewDependentPart> oldParts;
        std::vector<ViewDependentPart> newParts;
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoIncremental.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        double maxDist(0);
                        if (layerNo==0)
                            maxDist = 0.45;
                        else if (layerNo==1)
                            maxDist = 0.5;
                        std::vector<ViewDependentPart> oldPartsTmp;
                        std::vector<ViewDependentPart> newPartsTmp;
                        imageFilterer->identifyParts(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, (int)layerNo, true, maxDist,oldPartsTmp, newPartsTmp);
                        newParts.insert(newParts.end(), newPartsTmp.begin(), newPartsTmp.end());
                        oldParts.insert(oldParts.end(), oldPartsTmp.begin(), oldPartsTmp.end());
                    }
                }
            }
        }
        std::cout << "oldParts " << oldParts.size() << "\n";
        std::cout << "newParts " << newParts.size() << "\n";
        std::cout << "Compute statistics for " << newParts.size() << " octets (" << layerNo+2 << " layer)\n";
        hierarchy.get()->addParts(oldParts,layerNo);
        partSelector->selectParts(newParts, *hierarchy, layerNo+2);
        std::cout << "Dictionary size after clusterization: " << newParts.size() << "\n";
        hierarchy.get()->addNewParts(newParts, layerNo);

        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoIncremental.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        imageFilterer->computePartsImage(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, (int)layerNo, true);
                    }
                }
            }
        }
    }
    objectsInference.resize(datasetInfoIncremental.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            ObjectCompositionOctree object(config.compositionConfig);
            objectsInference[categoryNo].push_back(object);
            for (size_t imageNo=0;imageNo<datasetInfoIncremental.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                std::vector<ViewDependentPart> parts;
                //get parts of the 3rd layers
                imageFilterer->getLayerParts((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDepPartsFromLayerNo-1, parts, true);
                //move octets into 3D space and update octree representation of the object
                Mat34 cameraPose(datasetIncremental->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                objectsInference[categoryNo][objectNo].updatePCLGrid(parts, cameraPose);
            }
        }
    }
    ObjectCompositionOctree::setRealisationCounter(imageFilterer->getRealisationsNo()+10000);
    for (size_t layerNo=0;layerNo<(size_t)config.viewIndependentLayersNo;layerNo++){
        std::vector<ViewIndependentPart> vocabulary;
        for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> voc;
                objectsInference[categoryNo][objectNo].createNextLayerVocabulary((int)layerNo, voc);
                vocabulary.insert( vocabulary.end(), voc.begin(), voc.end() );
            }
        }
        std::vector<ViewIndependentPart> oldParts;
        std::vector<ViewIndependentPart> newParts;
        for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> oldPartsTmp;
                std::vector<ViewIndependentPart> newPartsTmp;
                double maxDist(0);
                if (layerNo==0)
                    maxDist = 4.004;
                else if (layerNo==1)
                    maxDist = 0.002;
                objectsInference[categoryNo][objectNo].identifyParts((int)layerNo, hierarchy.get()->viewIndependentLayers[layerNo], *hierarchy, maxDist,oldPartsTmp, newPartsTmp);
                newParts.insert(newParts.end(), newPartsTmp.begin(), newPartsTmp.end());
                oldParts.insert(oldParts.end(), oldPartsTmp.begin(), oldPartsTmp.end());
            }
        }

        std::cout << "oldParts " << oldParts.size() << "\n";
        std::cout << "newParts " << newParts.size() << "\n";

        hierarchy.get()->addParts(oldParts, (int)layerNo);
        partSelector->selectParts(newParts, *hierarchy, (int)layerNo+2);
        std::cout << "Dictionary size after clusterization: " << newParts.size() << "\n";
        hierarchy.get()->addNewParts(newParts, (int)layerNo);

        for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
                objectsInference[categoryNo][objectNo].updateVoxelsPose((int)layerNo, hierarchy.get()->viewIndependentLayers[layerNo], *hierarchy);
            }
        }
    }
    hierarchy->showInfo();
    // save hierarchy to file
    if(config.save2file){
        std::cout << "save 2 file\n";
        std::ofstream ofsHierarchy(config.filename2save);
        ofsHierarchy << *hierarchy;
        for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
                ofsHierarchy << objects[categoryNo][objectNo];
            }
        }
        for (size_t categoryNo=0;categoryNo<datasetInfoIncremental.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoIncremental.categories[categoryNo].objects.size();objectNo++){//for each object
                ofsHierarchy << objectsInference[categoryNo][objectNo];
            }
        }
        std::cout << "dfdf\n";
        ((NormalImageFilter*)imageFilterer)->mergeTrainAndInfResults();
        std::cout << "dfdf1\n";
        ((NormalImageFilter*)imageFilterer)->save2file(ofsHierarchy, false);
        //((NormalImageFilter*)imageFilterer)->save2file(ofsHierarchy, true);
        ofsHierarchy.close();
        std::cout << "saved\n";
    }
}

/// load hierarchy from the file
void HOP3DBham::load(std::string filename){
    std::ifstream ifsHierarchy(filename);
    ifsHierarchy >> *hierarchy;
    datasetTrain->getDatasetInfo(datasetInfoTrain);
    objects.resize(datasetInfoTrain.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
            ObjectCompositionOctree object(config.compositionConfig);
            objects[categoryNo].push_back(object);
            ifsHierarchy >> objects[categoryNo][objectNo];
        }
    }
    ((NormalImageFilter*)imageFilterer)->loadFromfile(ifsHierarchy, false);
    ifsHierarchy.close();
    /*for (auto part : hierarchy.get()->viewDependentLayers[0][0].group){
        part.print();
        getchar();
    }*/
    std::cout << "Loaded\n";
    hierarchy->showInfo();
    //visualization
    notifyVisualizer();
    std::cout << "Finished\n";
}

/// load inference results from the file
void HOP3DBham::loadInference(std::string filename){
    std::ifstream ifsInference(filename);
    std::cout << "Load inference results...\n";
    datasetTest->getDatasetInfo(datasetInfoTest);
    objectsInference.resize(datasetInfoTest.categories.size());
    std::map<std::string,int> objectsCoveragesGlob;
    int inferenceLayerNo=2;
    for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){//for each object
            ObjectCompositionOctree object(config.compositionConfig);
            objectsInference[categoryNo].push_back(object);
            ifsInference >> objectsInference[categoryNo][objectNo];
            if (inferenceLayerNo>2){
                std::vector<ViewIndependentPart::Part3D> partView;
                for (int overlapNo=0;overlapNo<3;overlapNo++){
                    std::vector<ViewIndependentPart::Part3D> partsViewTmp;
                    objectsInference[categoryNo][objectNo].getPartsRealisation(inferenceLayerNo-3, overlapNo, partsViewTmp);
                    partView.insert(partView.end(), partsViewTmp.begin(), partsViewTmp.end());
                }
                for (auto part : partView){
                    std::map<std::string,int> objectsCoverage;
                    getObjectsBuildFromPart(part.id, inferenceLayerNo, objectsCoverage);
                    for (auto &element : objectsCoverage){
                        auto it = objectsCoveragesGlob.find(element.first);
                        if (it != objectsCoverage.end())
                            objectsCoveragesGlob[element.first]+=element.second;
                        else
                            objectsCoveragesGlob[element.first]=element.second;
                    }
                }
            }
        }
    }
    ((NormalImageFilter*)imageFilterer)->loadFromfile(ifsInference, true);
    ifsInference.close();
    std::cout << "Loaded\n";
    if (inferenceLayerNo<3){
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTest.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        std::vector<PartCoordsEucl> partCoords;
                        imageFilterer->getParts3D(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, inferenceLayerNo, partCoords, false);
                        for (auto part : partCoords){
                            std::map<std::string,int> objectsCoverage;
                            getObjectsBuildFromPart(part.filterId, inferenceLayerNo, objectsCoverage);
                            for (auto &element : objectsCoverage){
                                auto it = objectsCoveragesGlob.find(element.first);
                                if (it != objectsCoverage.end())
                                    objectsCoveragesGlob[element.first]+=element.second;
                                else
                                    objectsCoveragesGlob[element.first]=element.second;
                            }
                        }
                    }
                }
            }
        }
    }
    double sumCov=0;
    for (auto &element : objectsCoveragesGlob){
        sumCov+=element.second;
        std::cout << element.first << "-> " << element.second << "\n";
    }
    for (auto &element : objectsCoveragesGlob)
        std::cout << element.first << "-> [%] " << 100.0*double(element.second)/sumCov << "\n";
    //visualization
#ifdef QVisualizerBuild
    if (config.useVisualization){
        createObjsFromParts(true);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        notify3Dmodels();
        createPartClouds(true);
    }
#endif
    std::cout << "Finished\n";
}

/// return object which are build from part id
void HOP3DBham::getObjectsBuildFromPart(int partId, int layerNo, std::map<std::string,int>& objectNames){
    objectNames.clear();
    for (size_t categoryNo=0;categoryNo<datasetInfoTrain.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoTrain.categories[categoryNo].objects.size();objectNo++){//for each object
            for (int overlapNo=0;overlapNo<3;overlapNo++){
                std::vector<ViewIndependentPart::Part3D> partsViewTmp;
                if (layerNo>2){
                    objects[categoryNo][objectNo].getPartsRealisation(layerNo-3, overlapNo, partsViewTmp);
                    for (auto &part : partsViewTmp){
                        if (partId == part.id){
                            std::string objectName = datasetInfoTrain.categories[categoryNo].objects[objectNo].name;
                            int coveragePart = (int)hierarchy.get()->viewIndependentLayers[layerNo-3][partId].cloud.size();
                            auto it = objectNames.find(objectName);
                            if (it != objectNames.end())
                                objectNames[objectName]+=coveragePart;
                            else
                                objectNames[objectName]=coveragePart;
                        }
                    }
                }
                else{
                    std::vector<PartCoordsEucl> partCoords;
                    for (size_t imageNo=0;imageNo<datasetInfoTrain.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        std::vector<PartCoordsEucl> partCoordsTmp;
                        imageFilterer->getParts3D(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, layerNo, partCoordsTmp, false);
                        partCoords.insert(partCoords.end(), partCoordsTmp.begin(), partCoordsTmp.end());
                    }
                    for (auto &part : partCoords){
                        if (partId == part.filterId&&hierarchy.get()->viewDependentLayers[layerNo-1][partId].group.size()<15){
                            std::string objectName = datasetInfoTrain.categories[categoryNo].objects[objectNo].name;
                            int coveragePart = 1;
                            std::cout << objectName << "-> " << coveragePart << " jjj\n";
                            auto it = objectNames.find(objectName);
                            if (it != objectNames.end())
                                objectNames[objectName]+=coveragePart;
                            else
                                objectNames[objectName]=coveragePart;
                        }
                    }
                }
            }
        }
    }
}

/// inference
void HOP3DBham::inference(void){
    if (hierarchy.get()->viewDependentLayers[0].size()==0)
        throw std::runtime_error("Train or load hierarchy first\n");
    datasetTest->getDatasetInfo(datasetInfoTest);
    int toLayerDep = (config.inferenceUpToLayer>config.viewDependentLayersNo) ? config.viewDependentLayersNo : config.inferenceUpToLayer;
    int toLayerIndep = (config.inferenceUpToLayer>config.viewDependentLayersNo) ? config.inferenceUpToLayer-config.viewDependentLayersNo : 0;
    std::cout << "Start inference dataset\n";
    for (int layerNo=0;layerNo<toLayerDep;layerNo++){
        if (layerNo==0){
            for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTest.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        cv::Mat image;
                        datasetTest->getDepthImage((int)categoryNo,(int)objectNo,(int)imageNo,image);
                        std::vector<hop3d::Octet> octetsTmp;
                        imageFilterer->computeOctets(image, (int)categoryNo, (int)objectNo, (int)imageNo, octetsTmp, true);
                        imageFilterer->getOctets((int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, octetsTmp, true);
                    }
                }
            }
        }
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfoTest.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        imageFilterer->computePartsImage(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, (int)layerNo, true);
                    }
                }
            }
        }
    }
    objectsInference.resize(datasetInfoTest.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            ObjectCompositionOctree object(config.compositionConfig);
            objectsInference[categoryNo].push_back(object);
            for (size_t imageNo=0;imageNo<datasetInfoTest.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                std::vector<ViewDependentPart> parts;
                //get parts of the 3rd layers
                imageFilterer->getLayerParts((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDepPartsFromLayerNo-1, parts, true);
                //move octets into 3D space and update octree representation of the object
                Mat34 cameraPose(datasetTest->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                objectsInference[categoryNo][objectNo].updatePCLGrid(parts, cameraPose);
            }
        }
    }
    //ObjectCompositionOctree::setRealisationCounter(imageFilterer->getRealisationsNo()+10000);
    std::map<std::string,int> objectsCoveragesGlob;
    for (size_t layerNo=0;layerNo<(size_t)toLayerIndep;layerNo++){
        for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> voc;
                objectsInference[categoryNo][objectNo].createNextLayerVocabulary((int)layerNo, voc);
            }
        }
        for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){//for each object
                objectsInference[categoryNo][objectNo].updateVoxelsPose((int)layerNo, hierarchy.get()->viewIndependentLayers[layerNo], *hierarchy);
                std::vector<ViewIndependentPart::Part3D> partView;
                for (int overlapNo=0;overlapNo<3;overlapNo++){
                    std::vector<ViewIndependentPart::Part3D> partsViewTmp;
                    objectsInference[categoryNo][objectNo].getPartsRealisation(1, overlapNo, partsViewTmp);
                    partView.insert(partView.end(), partsViewTmp.begin(), partsViewTmp.end());
                }
                for (auto part : partView){
                    std::map<std::string,int> objectsCoverage;
                    getObjectsBuildFromPart(part.id, 1, objectsCoverage);
                    for (auto &element : objectsCoverage){
                        auto it = objectsCoveragesGlob.find(element.first);
                        if (it != objectsCoverage.end())
                            objectsCoveragesGlob[element.first]+=element.second;
                        else
                            objectsCoveragesGlob[element.first]=element.second;
                    }
                }
            }
        }
    }
    double sumCov=0;
    for (auto &element : objectsCoveragesGlob){
        sumCov+=element.second;
        //std::cout << element.first << "-> " << element.second << "\n";
    }
    for (auto &element : objectsCoveragesGlob)
        std::cout << element.first << "-> [%] " << double(element.second)/sumCov << "\n";
    std::cout << "Inference finished\n";
    if(config.saveInference){
        std::cout << "Save to file...\n";
        std::ofstream ofsInference(config.filename2saveInference);
        for (size_t categoryNo=0;categoryNo<datasetInfoTest.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfoTest.categories[categoryNo].objects.size();objectNo++){//for each object
                ofsInference << objectsInference[categoryNo][objectNo];
            }
        }
        ((NormalImageFilter*)imageFilterer)->save2file(ofsInference, true);
        ofsInference.close();
        std::cout << "saved\n";
    }
    //visualization
#ifdef QVisualizerBuild
    if (config.useVisualization){
        createObjsFromParts(true);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        notify3Dmodels();
        createPartClouds(true);
    }
#endif
    std::cout << "Finished\n";
}

/// inference
void HOP3DBham::inference(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& images){
//void HOP3DBham::inference(std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Mat34>>& cameraFrames, int categoryNo, int objectNo){
    std::vector<std::pair<cv::Mat, Mat34>> cameraFramesImg;
    int imageNo=0;
    std::vector<std::string> names;
    for (const auto element : images){
        cv::Mat image;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tt;
        tt->sensor_origin_;
        Eigen::Vector4f origin = element.second->sensor_origin_;
        Eigen::Quaternionf rot = element.second->sensor_orientation_;
        Mat34 cameraPose(Eigen::Translation<double, 3>(origin(0),origin(1),origin(2))*Quaternion(rot.w(),rot.x(),rot.y(),rot.z()));
        BorisDataset::cloud2Image(element.second, image, *depthCameraModel);
        cameraFramesImg.push_back(std::make_pair(image,cameraPose));
        names.push_back(element.first);
        imageNo++;
    }
    inference(cameraFramesImg, names);
    hop3d::PointCloud cloud;
    /*
    getCloud("mrmuscle.pcd", cloud, true);

    Mat34 frameHop;
    getSensorFrame("mrmuscle.pcd", frameHop, true);

    // convert realisation graph
    hop3d::Hierarchy::IndexSetMap realisationsHop;
    getRealisationsGraph("mrmuscle.pcd", realisationsHop, true);

    // convert parts
    std::vector<hop3d::ViewIndependentPart::Part3D> partsHop;
    getPartsRealisation("mrmuscle.pcd", partsHop, true);

    // convert indices
    hop3d::HOP3D::PartsClouds indicesHop;
    getPartsRealisationCloud("mrmuscle.pcd", indicesHop, true);
    */
}

/// inference
void HOP3DBham::inference(std::vector<std::pair<cv::Mat, Mat34>>& cameraFrames, std::vector<std::string>& names){
    if (hierarchy.get()->viewDependentLayers[0].size()==0)
        throw std::runtime_error("Train or load hierarchy first\n");
    if (cameraFrames.size()!=names.size())
        throw std::runtime_error("cameraFrames vector and names vector should have the same size\n");
    inferenceData.clear();
    int frameNo=0;
    for (const auto& frame : cameraFrames){
        inferenceData.insert(names[frameNo],frame.first,frame.second);
        frameNo++;
    }

    int toLayerDep = (config.inferenceUpToLayer>config.viewDependentLayersNo) ? config.viewDependentLayersNo : config.inferenceUpToLayer;
    int toLayerIndep = (config.inferenceUpToLayer>config.viewDependentLayersNo) ? config.inferenceUpToLayer-config.viewDependentLayersNo : 0;
    std::cout << "Start inference depth images\n";
    //datasetTest->getDatasetInfo(datasetInfoTest);
    int categoryNo = 0;
    int objectNo = 0;
    for (int layerNo=0;layerNo<toLayerDep;layerNo++){
        if (layerNo==0){
            int imageNo=0;
            for (const auto& frame : cameraFrames){
                std::vector<hop3d::Octet> octetsTmp;
                imageFilterer->computeOctets(frame.first, categoryNo, objectNo, imageNo, octetsTmp, true);
                imageNo++;
            }
        }
        if (layerNo==1){
            int imageNo=0;
            for (size_t frameNo=0; frameNo<cameraFrames.size(); frameNo++){
                std::vector<hop3d::Octet> octetsTmp;
                imageFilterer->getOctets(categoryNo, objectNo, imageNo, *hierarchy, octetsTmp, true);
                imageNo++;
            }
        }
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (int imageNo=0; imageNo<(int)cameraFrames.size(); imageNo++){
                imageFilterer->computePartsImage(overlapNo, categoryNo, objectNo, imageNo, *hierarchy, layerNo, true);
            }
        }
    }
    objectsInference.clear();
    objectsInference.resize(categoryNo+1);
    std::cout << "Create object composition\n";
    ObjectCompositionOctree object(config.compositionConfig);
    objectsInference[categoryNo].push_back(object);
    for (int imageNo=0; imageNo<(int)cameraFrames.size(); imageNo++){
        std::vector<ViewDependentPart> parts;
        //get parts of the 3rd layers
        imageFilterer->getLayerParts(categoryNo, objectNo, imageNo, hierarchy.get()->viewDepPartsFromLayerNo-1, parts, true);
        //move octets into 3D space and update octree representation of the object
        objectsInference[categoryNo][objectNo].updatePCLGrid(parts, cameraFrames[imageNo].second);
    }
    //ObjectCompositionOctree::setRealisationCounter(imageFilterer->getRealisationsNo()+10000);
    std::map<std::string,int> objectsCoveragesGlob;
    for (size_t layerNo=0;layerNo<(size_t)toLayerIndep;layerNo++){
        std::vector<ViewIndependentPart> voc;
        objectsInference[categoryNo][objectNo].createNextLayerVocabulary((int)layerNo, voc);
        objectsInference[categoryNo][objectNo].updateVoxelsPose((int)layerNo, hierarchy.get()->viewIndependentLayers[layerNo], *hierarchy);
        std::vector<ViewIndependentPart::Part3D> partView;
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<ViewIndependentPart::Part3D> partsViewTmp;
            objectsInference[categoryNo][objectNo].getPartsRealisation(1, overlapNo, partsViewTmp);
            partView.insert(partView.end(), partsViewTmp.begin(), partsViewTmp.end());
        }
        for (auto part : partView){
            std::map<std::string,int> objectsCoverage;
            getObjectsBuildFromPart(part.id, 1, objectsCoverage);
            for (auto &element : objectsCoverage){
                auto it = objectsCoveragesGlob.find(element.first);
                if (it != objectsCoverage.end())
                    objectsCoveragesGlob[element.first]+=element.second;
                else
                    objectsCoveragesGlob[element.first]=element.second;
            }
        }
    }
    double sumCov=0;
    for (auto &element : objectsCoveragesGlob){
        sumCov+=element.second;
        //std::cout << element.first << "-> " << element.second << "\n";
    }
    for (auto &element : objectsCoveragesGlob)
        std::cout << element.first << "-> [%] " << double(element.second)/sumCov << "\n";
    std::cout << "Inference finished\n";
    /*if(config.saveInference){
        std::cout << "Save to file...\n";
        std::ofstream ofsInference(config.filename2saveInference);
        ofsInference << objectsInference[categoryNo][objectNo];
        ((NormalImageFilter*)imageFilterer)->save2file(ofsInference, true);
        ofsInference.close();
        std::cout << "saved\n";
    }*/
    //visualization
#ifdef QVisualizerBuild
    if (config.useVisualization){
        createObjsFromParts();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        notify3Dmodels();
        createPartClouds();
    }
#endif
    std::cout << "Finished\n";
}

/// notify visualizer
void HOP3DBham::notifyVisualizer(void){
#ifdef QVisualizerBuild
    if (config.useVisualization){
        notify(*hierarchy);
        createObjsFromParts(false);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        notify3Dmodels();
        createPartClouds(false);
        // draw parts coordinates
        /*std::vector<ViewIndependentPart::Part3D> parts;
        getPartsRealisation(0,0,0, parts);
        std::vector<Mat34> coords;
        for (auto &part : parts){
            coords.push_back(part.pose);
        }
        notify(coords);*/
    }
#endif
}

/// create objects from parts
void HOP3DBham::createObjsFromParts(bool inference){
    DatasetInfo* datasetInfo;
    Dataset* dataset;
    std::vector<std::vector<ObjectCompositionOctree>>* objComp;//objects compositions
    if (inference){
        datasetInfo = &datasetInfoTest;
        dataset = datasetTest.get();
        objComp = &objectsInference;
    }
    else{
        datasetInfo = &datasetInfoTrain;
        dataset = datasetTrain.get();
        objComp = &objects;
    }
    for (int layerNo=0;layerNo<config.viewDependentLayersNo+1;layerNo++){//create objects from parts
        for (size_t categoryNo=0;categoryNo<datasetInfo->categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo->categories[categoryNo].objects.size();objectNo++){//for each object
                for (size_t imageNo=0;imageNo<datasetInfo->categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                    std::vector<PartCoords> partCoords;
                    std::vector<PartCoordsEucl> partCoordsEucl;
                    for (int overlapNo=0; overlapNo<3; overlapNo++){
                        if (layerNo==0){
                            std::vector<PartCoords> partCoordsTmp;
                            imageFilterer->getResponseFilters(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, partCoordsTmp, inference);
                            partCoords.insert(partCoords.end(), partCoordsTmp.begin(), partCoordsTmp.end());
                        }
                        else{
                            std::vector<PartCoordsEucl> partCoordsEuclTmp;
                            imageFilterer->getParts3D(overlapNo,(int)categoryNo, (int)objectNo, (int)imageNo, layerNo, partCoordsEuclTmp, inference);
                            partCoordsEucl.insert(partCoordsEucl.end(), partCoordsEuclTmp.begin(), partCoordsEuclTmp.end());
                        }
                    }
                    Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                    std::vector<std::pair<int, Mat34>> filtersPoses;
                    if (layerNo<1){
                        for (auto& filterCoord : partCoords){
                            Vec3 point3d;
                            depthCameraModel.get()->getPoint(filterCoord.coords.u, filterCoord.coords.v, filterCoord.coords.depth, point3d);
                            Mat34 pointPose(Mat34::Identity());
                            pointPose.translation() = point3d;
                            pointPose = cameraPose * pointPose*filterCoord.offset;
                            filtersPoses.push_back(std::make_pair(filterCoord.filterId,pointPose));
                        }
                    }
                    else{
                        for (auto& filterCoord : partCoordsEucl){
                            Mat34 pointPose(Mat34::Identity());
                            pointPose.translation() = filterCoord.coords;
                            pointPose = cameraPose * pointPose*filterCoord.offset;
                            filtersPoses.push_back(std::make_pair(filterCoord.filterId,pointPose));
                        }
                    }
                    //compute object index
                    int objNo=0;
                    for (size_t catNo=0;catNo<categoryNo;catNo++){
                        objNo+=(int)datasetInfo->categories[catNo].objects.size();
                    }
                    notify(filtersPoses,(int)objNo,layerNo, inference);
                }
            }
        }
    }
    for (size_t categoryNo=0;categoryNo<datasetInfo->categories.size();categoryNo++){//for each category
        for (auto & object : (*objComp)[categoryNo]){
            int objectNo = 0;
            objectNo++;
            std::vector<ViewIndependentPart> objectParts;
            for (size_t i=0;i<hierarchy.get()->viewIndependentLayers.size();i++){
                object.getParts((int)i, objectParts);
                notify(objectParts, (int)(i+hierarchy.get()->viewDependentLayers.size()+1), inference);
                //std::cout << "(int)(i+hierarchy.get()->viewDependentLayers.size()+1) " < < (int)(i+hierarchy.get()->viewDependentLayers.size()+1) << "\n";
            }
        }
    }
}

/// create objects from parts
void HOP3DBham::createObjsFromParts(void){
    std::vector<std::vector<ObjectCompositionOctree>>* objComp = &objectsInference;//objects compositions
    int categoryNo=0;
    int objectNo=0;
    for (int layerNo=0;layerNo<config.viewDependentLayersNo+1;layerNo++){//create objects from parts
        int imageNo=0;
        for (auto & inputData : inferenceData.data){//for each depth image
            std::vector<PartCoords> partCoords;
            std::vector<PartCoordsEucl> partCoordsEucl;
            for (int overlapNo=0; overlapNo<3; overlapNo++){
                if (layerNo==0){
                    std::vector<PartCoords> partCoordsTmp;
                    imageFilterer->getResponseFilters(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, partCoordsTmp, true);
                    partCoords.insert(partCoords.end(), partCoordsTmp.begin(), partCoordsTmp.end());
                }
                else{
                    std::vector<PartCoordsEucl> partCoordsEuclTmp;
                    imageFilterer->getParts3D(overlapNo,(int)categoryNo, (int)objectNo, (int)imageNo, layerNo, partCoordsEuclTmp, true);
                    partCoordsEucl.insert(partCoordsEucl.end(), partCoordsEuclTmp.begin(), partCoordsEuclTmp.end());
                }
            }
            Mat34 cameraPose(inputData.second.cameraPose);
            std::vector<std::pair<int, Mat34>> filtersPoses;
            if (layerNo<1){
                for (auto& filterCoord : partCoords){
                    Vec3 point3d;
                    depthCameraModel.get()->getPoint(filterCoord.coords.u, filterCoord.coords.v, filterCoord.coords.depth, point3d);
                    Mat34 pointPose(Mat34::Identity());
                    pointPose.translation() = point3d;
                    pointPose = cameraPose * pointPose*filterCoord.offset;
                    filtersPoses.push_back(std::make_pair(filterCoord.filterId,pointPose));
                }
            }
            else{
                for (auto& filterCoord : partCoordsEucl){
                    Mat34 pointPose(Mat34::Identity());
                    pointPose.translation() = filterCoord.coords;
                    pointPose = cameraPose * pointPose*filterCoord.offset;
                    filtersPoses.push_back(std::make_pair(filterCoord.filterId,pointPose));
                }
            }
            //compute object index
            notify(filtersPoses,objectNo, layerNo, true);
            imageNo++;
        }
    }
    for (auto & object : (*objComp)[categoryNo]){
        std::vector<ViewIndependentPart> objectParts;
        std::cout << "object composition\n";
        for (size_t i=0;i<hierarchy.get()->viewIndependentLayers.size();i++){
            object.getParts((int)i, objectParts);
            notify(objectParts, (int)(i+hierarchy.get()->viewDependentLayers.size()+1), true);
            std::cout << "objectsParts.size(): " << objectParts.size() << "\n";
            //std::cout << "(int)(i+hierarchy.get()->viewDependentLayers.size()+1) " < < (int)(i+hierarchy.get()->viewDependentLayers.size()+1) << "\n";
        }
    }
}

/// get set of ids from hierarchy for the given input point
void HOP3DBham::getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, int u, int v, double depth, std::vector<int>& ids, bool inference) const{
    ids.clear();
    ViewDependentPart lastVDpart;
    imageFilterer->getPartsIds(overlapNo, categoryNo, objectNo, imageNo, u, v, depth, ids, lastVDpart, inference);
}

/// get set of ids from hierarchy for the given input point (view-independent layers)
void HOP3DBham::getPartsIds(int overlapNo, int categoryNo, int objectNo, const Mat34& cameraPose, const Vec3& point, std::vector<int>& ids, bool inference) const{
    if ((!std::isnan(point(0)))&&(!std::isnan(point(1)))){
        Vec3 pointGlob = (cameraPose*Vec4(point(0),point(1),point(2),1.0)).block<3,1>(0,0);
        if (inference)
            objectsInference[categoryNo][objectNo].getPartsIds(pointGlob, overlapNo, ids);
        else
            objects[categoryNo][objectNo].getPartsIds(pointGlob, overlapNo, ids);
    }
    else{
        std::vector<int> notUsed(3,-1);
        ids.insert(std::end(ids), std::begin(notUsed), std::end(notUsed));
    }
}

/// get realisations ids
void HOP3DBham::getRealisationsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, const Mat34& cameraPose, int u, int v, double depth, std::vector<int>& ids, bool inference) const{
    ids.clear();
    ViewDependentPart lastVDpart;
    std::vector<int> idsTmp;
    imageFilterer->getRealisationsIds(overlapNo, categoryNo, objectNo, imageNo, u, v, depth, idsTmp, lastVDpart, inference);
    ids.insert(ids.end(),idsTmp.begin(),idsTmp.end());
    //std::cout << ids[0] << ", " << ids[1] << ", " << ids[2] << "\n";
    /// view independent ids
    //Mat34 cameraPose(datasetTrain->getCameraPose(categoryNo, objectNo, imageNo));
    Vec3 point(Vec3(NAN,NAN,NAN));
    depthCameraModel.get()->getPoint(u, v, depth, point);
    if ((!std::isnan(point(0)))&&(!std::isnan(point(1)))){
        point = (cameraPose*Vec4(point(0),point(1),point(2),1.0)).block<3,1>(0,0);
        idsTmp.clear();
        if (inference)
            objectsInference[categoryNo][objectNo].getRealisationsIds(point, overlapNo, idsTmp);
        else
            objects[categoryNo][objectNo].getRealisationsIds(point, overlapNo, idsTmp);
        ids.insert(ids.end(),idsTmp.begin(),idsTmp.end());
    }
    else{
        std::vector<int> notUsed(3,-1);
        ids.insert(std::end(ids), std::begin(notUsed), std::end(notUsed));
    }
}

/// get points realisation for the cloud
void HOP3DBham::getPointsModels(int overlapNo, int categoryNo, int objectNo, int imageNo, const cv::Mat& depthImage, const Mat34& cameraPose, hop3d::PartsCloud& cloudParts, bool inference) const{
    PointCloudUV cloudUV;
    imageFilterer->getCloud(depthImage,cloudUV);
    for (const auto &pointuv : cloudUV){
        std::vector<int> ids;
        getPartsIds(overlapNo, categoryNo, objectNo, imageNo, pointuv.u, pointuv.v, pointuv.position(2), ids, inference);
        PointPart pointPart;
        pointPart.position=pointuv.position;
        for (int i=0;i<(int)ids.size();i++){
            pointPart.partsIds.push_back(std::make_pair(i,ids[i]));
        }
        int idsSize = (int)ids.size();
        ids.clear();
        getPartsIds(overlapNo, categoryNo, objectNo, cameraPose, pointuv.position, ids, inference);
        for (int i=0;i<(int)ids.size();i++){
            pointPart.partsIds.push_back(std::make_pair(i+idsSize,ids[i]));
        }
        cloudParts.push_back(pointPart);
    }
}

/// assign random colors to parts
void HOP3DBham::randomColors(int layersNo){
    colors.resize(layersNo);
    colors[0].resize(hierarchy.get()->firstLayer.size());
    std::uniform_real_distribution<double> uniformDist(0.0,1.0);
    std::default_random_engine engine;
    for (int layerNo=0;layerNo<(int)hierarchy.get()->viewDependentLayers.size();layerNo++){
        colors[layerNo+1].resize(hierarchy.get()->viewDependentLayers[layerNo].size());
    }
    for (int layerNo=0;layerNo<(int)hierarchy.get()->viewIndependentLayers.size();layerNo++){
        colors[layerNo+(int)hierarchy.get()->viewDependentLayers.size()+1].resize(hierarchy.get()->viewIndependentLayers[layerNo].size());
    }
    for (size_t layerNo=0;layerNo<colors.size();layerNo++){
        for (size_t partNo=0;partNo<colors[layerNo].size();partNo++){
            std::array<double,4> color = {uniformDist(engine), uniformDist(engine), uniformDist(engine), 1.0};
            colors[layerNo][partNo] = color;
        }
    }
}

/// create part-coloured point clouds
void HOP3DBham::createPartClouds(bool inference){
    /// clouds for the first layer
    int layersNo=6;//(int)hierarchy.get()->viewDependentLayers.size()+(int)hierarchy.get()->viewIndependentLayers.size()+1;
    if (colors.size()==0){
        randomColors(layersNo);
    }
    DatasetInfo* datasetInfo;
    Dataset* dataset;
    //std::vector<std::vector<ObjectCompositionOctree>>* objComp;//objects compositions
    if (inference){
        datasetInfo = &datasetInfoTest;
        dataset = datasetTest.get();
       // objComp = &objectsInference;
    }
    else{
        datasetInfo = &datasetInfoTrain;
        dataset = datasetTrain.get();
       // objComp = &objects;
    }
    int overlapsNo=4;
    /// point clouds (index: overlapNo->layerNo->objectNo)
    std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>> cloudsObj(overlapsNo, std::vector<std::vector<hop3d::PointCloudRGBA>>(layersNo)); // vector of coloured objects
    for (int overlapNo=0; overlapNo<overlapsNo; overlapNo++){
        for (size_t categoryNo=0;categoryNo<datasetInfo->categories.size();categoryNo++){
            for (size_t objectNo=0;objectNo<datasetInfo->categories[categoryNo].objects.size();objectNo++){
                std::vector<hop3d::PointCloudRGBA> objsTmp(layersNo);
                for (size_t imageNo=0;imageNo<datasetInfo->categories[categoryNo].objects[objectNo].images.size();imageNo++){
                    hop3d::PartsCloud cloudParts;
                    if (overlapNo<3){
                        cv::Mat depthImage;
                        dataset->getDepthImage((int)categoryNo, (int)objectNo, (int)imageNo, depthImage);
                        Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                        getPointsModels(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, depthImage, cameraPose, cloudParts, inference);
                        for (auto &pointPart : cloudParts){
                            for (int el=0;el<(int)pointPart.partsIds.size();el++){
                                int layNo=pointPart.partsIds[el].first;
                                PointColor pointRGBA(pointPart.position,std::array<double,4>({0.0,0.0,0.0,1.0}));
                                if (pointPart.partsIds[layNo].second>=0){
                                    pointRGBA.color = colors[layNo][pointPart.partsIds[layNo].second];
                                }
                                if (!std::isnan(pointPart.position(0))){
                                    Mat34 pointCam(Quaternion(1,0,0,0)*Eigen::Translation<double, 3>(pointPart.position(0),pointPart.position(1),pointPart.position(2)));
                                    Mat34 pointWorld = cameraPose*pointCam;
                                    pointRGBA.position(0)=pointWorld(0,3); pointRGBA.position(2)=pointWorld(2,3); pointRGBA.position(1)=pointWorld(1,3);
                                    if (layNo<3)
                                        pointRGBA.position(1)+=0.2*double(imageNo);
                                    objsTmp[layNo].push_back(pointRGBA);
                                }
                            }
                        }
                    }
                    else{
                        for (int layNo=0;layNo<layersNo;layNo++){
                            //compute object index
                            int objNo=0;
                            for (size_t catNo=0;catNo<categoryNo;catNo++){
                                objNo+=(int)datasetInfo->categories[catNo].objects.size();
                            }
                            for (int pointNo=0;pointNo<(int)cloudsObj[0][layNo][objNo].size();pointNo++){
                                PointColor pointRGBA(cloudsObj[0][layNo][objNo][pointNo].position,std::array<double,4>({0.0,0.0,0.0,1.0}));
                                int colorsNo=0;
                                for (int overNo=0;overNo<3;overNo++){
                                    if (cloudsObj[overNo][layNo][objNo][pointNo].color[0]>0||cloudsObj[overNo][layNo][objNo][pointNo].color[1]>0||cloudsObj[overNo][layNo][objNo][pointNo].color[2]>0){
                                        colorsNo++;
                                        for (int colorId=0;colorId<3;colorId++)
                                            pointRGBA.color[colorId]+=cloudsObj[overNo][layNo][objNo][pointNo].color[colorId];
                                    }
                                }
                                if (colorsNo>0){
                                    for (int colorId=0;colorId<3;colorId++)
                                        pointRGBA.color[colorId]/=double(colorsNo);
                                }
                                objsTmp[layNo].push_back(pointRGBA);
                            }
                        }
                    }
                }
                for (size_t layerNo=0;layerNo<objsTmp.size();layerNo++){
                    cloudsObj[overlapNo][layerNo].push_back(objsTmp[layerNo]);
                }
            }
        }
    }
    notify(cloudsObj, inference);
    createPartObjects();
}

/// create part-coloured point clouds
void HOP3DBham::createPartClouds(void){
    /// clouds for the first layer
    int layersNo=6;//(int)hierarchy.get()->viewDependentLayers.size()+(int)hierarchy.get()->viewIndependentLayers.size()+1;
    if (colors.size()==0){
        randomColors(layersNo);
    }
    int overlapsNo=4;
    /// point clouds (index: overlapNo->layerNo->objectNo)
    std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>> cloudsObj(overlapsNo, std::vector<std::vector<hop3d::PointCloudRGBA>>(layersNo)); // vector of coloured objects
    size_t categoriesNo = 1; size_t objectsNo = 1;
    for (int overlapNo=0; overlapNo<overlapsNo; overlapNo++){
        for (size_t categoryNo=0;categoryNo<categoriesNo;categoryNo++){
            for (size_t objectNo=0;objectNo<objectsNo;objectNo++){
                std::vector<hop3d::PointCloudRGBA> objsTmp(layersNo);
                int imageNo = 0;
                for (auto & inputData : inferenceData.data){
                    hop3d::PartsCloud cloudParts;
                    if (overlapNo<3){
                        cv::Mat depthImage(inputData.second.depthImage);
                        Mat34 cameraPose(inputData.second.cameraPose);
                        getPointsModels(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, depthImage, cameraPose, cloudParts, true);
                        for (auto &pointPart : cloudParts){
                            for (int el=0;el<(int)pointPart.partsIds.size();el++){
                                int layNo=pointPart.partsIds[el].first;
                                PointColor pointRGBA(pointPart.position,std::array<double,4>({0.0,0.0,0.0,1.0}));
                                if (pointPart.partsIds[layNo].second>=0){
                                    pointRGBA.color = colors[layNo][pointPart.partsIds[layNo].second];
                                }
                                if (!std::isnan(pointPart.position(0))){
                                    Mat34 pointCam(Quaternion(1,0,0,0)*Eigen::Translation<double, 3>(pointPart.position(0),pointPart.position(1),pointPart.position(2)));
                                    Mat34 pointWorld = cameraPose*pointCam;
                                    pointRGBA.position(0)=pointWorld(0,3); pointRGBA.position(2)=pointWorld(2,3); pointRGBA.position(1)=pointWorld(1,3);
                                    if (layNo<3)
                                        pointRGBA.position(1)+=0.2*double(imageNo);
                                    objsTmp[layNo].push_back(pointRGBA);
                                }
                            }
                        }
                    }
                    else{
                        for (int layNo=0;layNo<layersNo;layNo++){
                            //compute object index
                            int objNo=0;
                            for (int pointNo=0;pointNo<(int)cloudsObj[0][layNo][objNo].size();pointNo++){
                                PointColor pointRGBA(cloudsObj[0][layNo][objNo][pointNo].position,std::array<double,4>({0.0,0.0,0.0,1.0}));
                                int colorsNo=0;
                                for (int overNo=0;overNo<3;overNo++){
                                    if (cloudsObj[overNo][layNo][objNo][pointNo].color[0]>0||cloudsObj[overNo][layNo][objNo][pointNo].color[1]>0||cloudsObj[overNo][layNo][objNo][pointNo].color[2]>0){
                                        colorsNo++;
                                        for (int colorId=0;colorId<3;colorId++)
                                            pointRGBA.color[colorId]+=cloudsObj[overNo][layNo][objNo][pointNo].color[colorId];
                                    }
                                }
                                if (colorsNo>0){
                                    for (int colorId=0;colorId<3;colorId++)
                                        pointRGBA.color[colorId]/=double(colorsNo);
                                }
                                objsTmp[layNo].push_back(pointRGBA);
                            }
                        }
                    }
                    imageNo++;
                }
                for (size_t layerNo=0;layerNo<objsTmp.size();layerNo++){
                    cloudsObj[overlapNo][layerNo].push_back(objsTmp[layerNo]);
                }
            }
        }
    }
    notify(cloudsObj, true);
    createPartObjects();
}

/// convert parts map to parts cloud
void HOP3DBham::convertPartsMap2PartsCloud(const Hierarchy::IndexSeqMap& points2parts, PartsClouds& partsCloud) const{
    for(auto &pointMap : points2parts){
        std::uint32_t pointIdx = pointMap.first;
        std::vector<std::uint32_t> partsIds = pointMap.second;
        for (auto &partId : partsIds){
            partsCloud[partId].insert(pointIdx);
        }
    }
}

hop3d::HOP3D* hop3d::createHOP3DBham(void) {
    hop3dBham.reset(new HOP3DBham());
    return hop3dBham.get();
}

hop3d::HOP3D* hop3d::createHOP3DBham(std::string config) {
    hop3dBham.reset(new HOP3DBham(config));
    return hop3dBham.get();
}
