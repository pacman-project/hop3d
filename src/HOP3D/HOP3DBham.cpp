#include "hop3d/HOP3D/HOP3DBham.h"
#include <random>
#include <thread>
#include <chrono>

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
    std::string filename = configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID())
		throw std::runtime_error("unable to load unbiased stats builder config file: " + filename);
    tinyxml2::XMLElement * group = config.FirstChildElement( "Hierarchy" );
    group->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
    if (verbose == 1) {
        std::cout << "Load HOP3DBham parameters...\n";
    }
    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewDependentLayersNo", &viewDependentLayersNo);
    group->FirstChildElement( "parameters" )->QueryIntAttribute("viewIndependentLayersNo", &viewIndependentLayersNo);

    group->FirstChildElement( "save2file" )->QueryBoolAttribute("save2file", &save2file);
    filename2save = group->FirstChildElement( "save2file" )->Attribute( "filename2save" );

    statsConfig = (config.FirstChildElement( "StatisticsBuilder" )->Attribute( "configFilename" ));
    config.FirstChildElement( "PartSelector" )->QueryIntAttribute("selectorType", &partSelectorType);
    selectorConfig = (config.FirstChildElement( "PartSelector" )->Attribute( "configFilename" ));
    filtererConfig = (config.FirstChildElement( "Filterer" )->Attribute( "configFilename" ));
    compositionConfig = (config.FirstChildElement( "ObjectComposition" )->Attribute( "configFilename" ));
    cameraConfig = (config.FirstChildElement( "CameraModel" )->Attribute( "configFilename" ));
    datasetConfig = (config.FirstChildElement( "Dataset" )->Attribute( "configFilename" ));

    config.FirstChildElement( "Filterer" )->QueryIntAttribute("filterType", &filterType);
    config.FirstChildElement( "Dataset" )->QueryIntAttribute("datasetType", &datasetType);
}

/// get set of ids from hierarchy for the given input point
/*void HOP3DBham::getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const{
    int categoryNo, objectNo, imageNo = 0;
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getPartsIds(categoryNo, objectNo, imageNo, u, v, ids);
}*/

/// get training dataset info
void HOP3DBham::getDatasetInfo(hop3d::DatasetInfo& _dataset) const{
    dataset->getDatasetInfo(_dataset);
}

/// returns paths for each cloud/image
void HOP3DBham::getCloudPaths(std::vector<std::string>& paths) const{
    paths.clear();
    hop3d::DatasetInfo info;
    dataset->getDatasetInfo(info);
    for (const auto &category : info.categories)
        for (const auto &object : category.objects)
            for (const auto &path : object.fullPaths)
                paths.push_back(path);
}

/// get cloud from dataset
void HOP3DBham::getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud) const{
    hop3d::PointCloudUV cloudUV;
    cv::Mat depthImage;
    dataset->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    imageFilterer->getCloud(depthImage, cloudUV);
    cloud.clear();
    cloud.reserve(cloudUV.size());
    for (auto &point : cloudUV){
        hop3d::PointNormal p3d;
        p3d=point;
        cloud.push_back(p3d);
    }
    //transform cloud to global frame
    /*Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
    for (auto &point : cloud){
        point.position = (cameraPose * Vec4(point.position(0),point.position(1),point.position(2),1.0)).block<3,1>(0,3);
        point.normal = cameraPose.rotation()*point.normal;
    }*/
}

/// get cloud from dataset
void HOP3DBham::getCloud(const std::string& path, hop3d::PointCloud& cloud) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getCloud(categoryNo, objectNo, imageNo, cloud);
}

/// get number of points in the point cloud
size_t HOP3DBham::getNumOfPoints(int categoryNo, int objectNo, int imageNo) const {
    return dataset->getNumOfPoints(categoryNo, objectNo, imageNo);
}

/// get point from the point cloud
/*void HOP3DBham::getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const{
    dataset->getPoint(categoryNo, objectNo, imageNo, pointNo, point);
}*/

/// get camera pose
void HOP3DBham::getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose) const{
    cameraPose = dataset->getCameraPose(categoryNo, objectNo, imageNo);
}

/// get camera pose
void HOP3DBham::getSensorFrame(const std::string& path, Mat34& cameraPose) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    cameraPose = dataset->getCameraPose(categoryNo, objectNo, imageNo);
}

/// get hierarchy graph
void HOP3DBham::getHierarchy(Hierarchy::IndexSeqMap& hierarchyGraph) const{
    hierarchy.get()->computeGraph(hierarchyGraph);
}

/// get parts realization
void HOP3DBham::getPartsRealisation(int categoryNo, int objectNo, int imageNo, std::vector<ViewIndependentPart::Part3D>& parts) const{
    parts.clear();
    Mat34 cameraPose(dataset->getCameraPose(categoryNo, objectNo, imageNo));
    for (int layerNo = 0;layerNo<(int)hierarchy.get()->viewDependentLayers.size()+1;layerNo++){
        std::vector<ViewDependentPart> partsView;
        for (int overlapNo=0;overlapNo<3;overlapNo++){
            std::vector<ViewDependentPart> partsViewTmp;
            imageFilterer->getPartsRealisation(overlapNo, categoryNo, objectNo, imageNo,layerNo,partsViewTmp);
            partsView.insert(partsView.end(), partsViewTmp.begin(), partsViewTmp.end());
        }
        for (auto& part : partsView){
            ViewIndependentPart::Part3D part3D;
            part3D.id = ((layerNo)*10000)+part.id;
            //Vec3 point3d;
            //depthCameraModel.get()->getPoint(part.location.u, part.location.v, part.location.depth, point3d);
            Mat34 pointPose(Mat34::Identity());
            pointPose.translation() = part.locationEucl;
            part3D.pose = cameraPose * pointPose*part.offset;
            part3D.realisationId = part.realisationId;
            parts.push_back(part3D);
        }
    }
}

/// get parts realization
void HOP3DBham::getPartsRealisationCloud(int categoryNo, int objectNo, int imageNo, PartsClouds& parts) const{
    parts.clear();
    Hierarchy::IndexSeqMap points2parts;
    getCloud2PartsMap(categoryNo, objectNo, imageNo, points2parts);
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
void HOP3DBham::getRealisationsGraph(const std::string& path, Hierarchy::IndexSetMap& realisationsGraph) const{
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getRealisationsGraph(categoryNo, objectNo, imageNo, realisationsGraph);
}

/// get realisations graph
void HOP3DBham::getRealisationsGraph(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSetMap& realisationsGraph) const{
    PointCloudUV cloud;
    cv::Mat depthImage;
    dataset->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    imageFilterer->getCloud(depthImage, cloud);
    for (auto &point : cloud){
        std::vector<int> ids;
        getRealisationsIds(categoryNo,objectNo,imageNo,point.u,point.v,point.position(2),ids);
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
            idPrev=partId;
            idNo++;
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
void HOP3DBham::getPartsRealisationCloud(const std::string& path, PartsClouds& parts) const{
    parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getPartsRealisationCloud(categoryNo, objectNo, imageNo, parts);
}

/// get maps from point to part realisation
void HOP3DBham::getCloud2PartsMap(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSeqMap& points2parts) const{
    points2parts.clear();
    PointCloudUV cloud;
    cv::Mat depthImage;
    dataset->getDepthImage(categoryNo, objectNo, imageNo, depthImage);
    imageFilterer->getCloud(depthImage, cloud);
    std::uint32_t pointIdx=0;
    for (auto &point : cloud){
        std::vector<int> ids;
        getRealisationsIds(categoryNo,objectNo,imageNo,point.u,point.v, point.position(2),ids);
        std::vector<std::uint32_t> idsParts;
        for (size_t layerNo=0;layerNo<3;layerNo++){
            if (ids[layerNo]>=0){
                idsParts.push_back((std::uint32_t)ids[layerNo]);
                //idsParts.push_back((((std::uint32_t)layerNo)*10000)+(std::uint32_t)ids[layerNo]);//for model ids
                //std::cout << "point " << pointIdx << " -> " << (std::uint32_t)ids[layerNo] << " lay no " << layerNo << "\n";
            }
        }
        points2parts.insert(std::make_pair(pointIdx,idsParts));
        pointIdx++;
    }
}

/// get maps from point to part realisation
void HOP3DBham::getCloud2PartsMap(const std::string& path, Hierarchy::IndexSeqMap& points2parts) const{
    points2parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getCloud2PartsMap(categoryNo, objectNo, imageNo, points2parts);
}

/// get parts realization
void HOP3DBham::getPartsRealisation(const std::string& path, std::vector<ViewIndependentPart::Part3D>& parts) const{
    parts.clear();
    int categoryNo(0), objectNo(0), imageNo(0);
    dataset->translateString(path, categoryNo, objectNo, imageNo);
    getPartsRealisation(categoryNo, objectNo, imageNo, parts);
}

/// learining from the dataset
void HOP3DBham::learn(){
    imageFilterer->getFilters(hierarchy.get()->firstLayer);
    hop3d::ViewDependentPart::Seq dictionary;
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
                        imageFilterer->getOctets((int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, octetsTmp);
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
        /*if (layerNo==1){
            for (auto &word : dictionary){
                word.print();
            }
            ViewDependentPart pp(dictionary[1]);
            PointsSecondLayer points;
            ViewDependentPart::createPointsMatrix(pp,hierarchy.get()->viewDependentLayers[0],0,points);
            notify(points);
        }*/
        std::cout << "Dictionary size after clusterization: " << dictionary.size() << "\n";
        hierarchy.get()->viewDependentLayers[layerNo]=dictionary;
    }
    //represent/explain all images in parts from i-th layer
    for (size_t layerNo=0; layerNo< hierarchy.get()->viewDependentLayers.size();layerNo++){
        for (int overlapNo=0; overlapNo<3; overlapNo++){
            for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){
                for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){
                    for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                        imageFilterer->computePartsImage(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, *hierarchy, (int)layerNo);
                    }
                }
            }
        }
    }
    //std::vector< std::set<int>> clusters;
    objects.resize(datasetInfo.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
            std::cout << "Create object composition\n";
            ObjectCompositionOctree object(config.compositionConfig);
            objects[categoryNo].push_back(object);
            for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                //int layerNo=3;
                std::vector<ViewDependentPart> parts;
                //get parts of the 3rd layers
                imageFilterer->getLayerParts((int)categoryNo, (int)objectNo, (int)imageNo, hierarchy.get()->viewDepPartsFromLayerNo-1, parts);
                //move octets into 3D space and update octree representation of the object
                Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
                objects[categoryNo][objectNo].updatePCLGrid(parts, cameraPose);
            }
            /*std::vector< std::set<int>> clustersTmp;
            //std::cout << "get clusters\n";
            objects[categoryNo][objectNo].getClusters(0,clustersTmp);
            std::vector<ViewIndependentPart> VIparts;
            objects[categoryNo][objectNo].getParts(-1,VIparts);
            //std::cout << " clusters size: " << clustersTmp.size() << "\n";
            std::cout << "finish get clusters\n";
            clusters.reserve( clusters.size() + clustersTmp.size() ); // preallocate memory
            clusters.insert( clusters.end(), clustersTmp.begin(), clustersTmp.end() );*/
        }
    }
    //std::cout << "clusters size: " << clusters.size() << "\n";
    //std::vector<ViewIndependentPart> vocabulary;
    //std::cout << "create unique clusters:\n";
    //partSelector->createUniqueClusters(clusters, vocabulary, *hierarchy);
    std::vector<ViewIndependentPart> vocabulary;
    for (size_t layerNo=0;layerNo<2/*config.viewIndependentLayersNo*/;layerNo++){
        vocabulary.clear();
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> voc;
                objects[categoryNo][objectNo].createNextLayerVocabulary((int)layerNo, *hierarchy, voc);
                vocabulary.insert( vocabulary.end(), voc.begin(), voc.end() );
            }
        }
        std::cout << layerNo+4 << " layer init vocabulary size: " << vocabulary.size() << "\n";
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        partSelector->selectParts(vocabulary, int(layerNo+4));
        std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
        std::cout << "Clusterization took = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
        std::cout << "Dictionary size (" << layerNo+4 << "-th layer): " << vocabulary.size() << "\n";
        /// First view-independent layer (three and a half layer)
        hierarchy.get()->viewIndependentLayers[layerNo]=vocabulary;
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
                objects[categoryNo][objectNo].updateVoxelsPose((int)layerNo, vocabulary);
            }
        }
    }
//    for (auto & word : vocabulary){
//        word.print();
//        getchar();
//    }
    /*for (int layerNo=0;layerNo<config.viewIndependentLayersNo-1;layerNo++){
        ///select part for ith layer
        vocabulary.clear();
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
                std::vector<ViewIndependentPart> vocab;
                objects[categoryNo][objectNo].createNextLayerVocabulary(layerNo+1, *hierarchy, vocab);
                vocabulary.insert(vocabulary.end(),vocab.begin(), vocab.end());
            }
        }
        //compute statistics
        //select parts
        std::cout << "initial " << layerNo+5 << "th layer vacabulary size: " << vocabulary.size() << "\n";
        std::cout << "Compute statistics for " << vocabulary.size() << " octets (" << layerNo + 5 << "-th layer)\n";
        ViewIndependentPart::Seq newVocabulary;
        statsBuilder->computeStatistics(vocabulary, layerNo+5, 0, newVocabulary);
        std::cout << "vocabulary size after statistics: " << newVocabulary.size() << "\n";
        partSelector->selectParts(newVocabulary, *hierarchy, layerNo+5);
        std::cout << "Dictionary size (" << layerNo+5 << "-th layer): " << newVocabulary.size() << "\n";
        hierarchy.get()->viewIndependentLayers[layerNo+1]=newVocabulary;
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (auto & object : objects[categoryNo]){
                object.updateIds(layerNo+1, hierarchy.get()->viewIndependentLayers[layerNo+1], *hierarchy);
            }
        }
    }*/
    // save hierarchy to file
    if(config.save2file){
        std::ofstream ofsHierarchy(config.filename2save);
        ofsHierarchy << *hierarchy;
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
                ofsHierarchy << objects[categoryNo][objectNo];
            }
        }
        ((NormalImageFilter*)imageFilterer)->save2file(ofsHierarchy);
        ofsHierarchy.close();
    }
    //visualization
    notify(*hierarchy);
    for (int layerNo=0;layerNo<config.viewDependentLayersNo+1;layerNo++){//create objects from parts
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
                for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){//for each depth image
                    std::vector<PartCoords> partCoords;
                    std::vector<PartCoordsEucl> partCoordsEucl;
                    for (int overlapNo=0; overlapNo<3; overlapNo++){
                        if (layerNo==0){
                            std::vector<PartCoords> partCoordsTmp;
                            imageFilterer->getResponseFilters(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, partCoordsTmp);
                            partCoords.insert(partCoords.end(), partCoordsTmp.begin(), partCoordsTmp.end());
                        }
                        else{
                            std::vector<PartCoordsEucl> partCoordsEuclTmp;
                            imageFilterer->getParts3D(overlapNo,(int)categoryNo, (int)objectNo, (int)imageNo, layerNo, partCoordsEuclTmp);
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
                            if (layerNo>0){
                                pointPose = cameraPose * pointPose*filterCoord.offset;
                                //std::cout << "layer no " << layerNo << filterCoord.offset.matrix() << "\n";
                                //std::cout << filterCoord.offset.matrix() << "\n";
                            }
                            else
                                pointPose = cameraPose * pointPose;
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
                    notify(filtersPoses,(int)objectNo,layerNo);
                }
            }
        }
    }
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (auto & object : objects[categoryNo]){
            std::vector<ViewIndependentPart> objectParts;
            for (size_t i=0;i<2;i++){//hierarchy.get()->viewIndependentLayers.size()-2;i++){
                object.getParts((int)i, objectParts);
                notify(objectParts, (int)(i+hierarchy.get()->viewDependentLayers.size()+1));
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    notify3Dmodels();
    createPartClouds();
    Hierarchy::IndexSeqMap hierarchyGraph;
    getHierarchy(hierarchyGraph);
    std::vector<ViewIndependentPart::Part3D> parts;
    getPartsRealisation(0,0,0, parts);
    /*for (auto &part : parts){
        std::cout << "part id " << part.id << "\n";
        std::cout << "part realisation id " << part.realisationId << "\n";
        std::cout << "part pose\n" << part.pose.matrix() << "\n";
    }*/
    Hierarchy::IndexSeqMap points2parts;
    getCloud2PartsMap(0,0,0, points2parts);
    PartsClouds partsCloud;
    getPartsRealisationCloud(0,0,0,partsCloud);
    Hierarchy::IndexSetMap realisationsGraph;
    getRealisationsGraph(0,0,0, realisationsGraph);
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
    std::cout << "Finished\n";
}

/// load hierarchy from the file
void HOP3DBham::load(std::string filename){
    std::ifstream ifsHierarchy(filename);
    std::cout << "Load hierarchy...";
    ifsHierarchy >> *hierarchy;
    std::cout << "Loaded\n";
    dataset->getDatasetInfo(datasetInfo);
    objects.resize(datasetInfo.categories.size());
    for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){//for each object
            ObjectCompositionOctree object(config.compositionConfig);
            objects[categoryNo].push_back(object);
            ifsHierarchy >> objects[categoryNo][objectNo];
        }
    }
    ((NormalImageFilter*)imageFilterer)->loadFromfile(ifsHierarchy);
    ifsHierarchy.close();
    //visualization
    notify(*hierarchy);
    /*for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){//for each category
        for (auto & object : objects[categoryNo]){
            std::vector<ViewIndependentPart> objectParts;
            for (size_t i=0;i<1;i++){//hierarchy.get()->viewIndependentLayers.size()-2;i++){
                object.getParts((int)i, objectParts);
                notify(objectParts, int(i+hierarchy.get()->viewDependentLayers.size()+1));
            }
        }
    }
    notify3Dmodels();*/
    Hierarchy::IndexSeqMap hierarchyGraph;
    getHierarchy(hierarchyGraph);
    std::vector<ViewIndependentPart::Part3D> parts;
    getPartsRealisation(0,0,0, parts);
    /*for (auto &part : parts){
        std::cout << "part id " << part.id << "\n";
        std::cout << "part realisation id " << part.realisationId << "\n";
        std::cout << "part pose\n" << part.pose.matrix() << "\n";
    }*/
    Hierarchy::IndexSeqMap points2parts;
    getCloud2PartsMap(0,0,0, points2parts);
    PartsClouds partsCloud;
    getPartsRealisationCloud(0,0,0,partsCloud);
    Hierarchy::IndexSetMap realisationsGraph;
    getRealisationsGraph(0,0,1, realisationsGraph);
    //createPartClouds();
    std::cout << "Finished\n";
}

/// get set of ids from hierarchy for the given input point
void HOP3DBham::getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, int u, int v, double depth, std::vector<int>& ids) const{
    ids.clear();
    ViewDependentPart lastVDpart;
    imageFilterer->getPartsIds(overlapNo, categoryNo, objectNo, imageNo, u, v, depth, ids, lastVDpart);
}

/// get set of ids from hierarchy for the given input point (view-independent layers)
void HOP3DBham::getPartsIds(int categoryNo, int objectNo, int imageNo, const Vec3& point, std::vector<int>& ids) const{
    Mat34 cameraPose(dataset->getCameraPose(categoryNo, objectNo, imageNo));
    if ((!std::isnan(point(0)))&&(!std::isnan(point(1)))){
        Vec3 pointGlob = (cameraPose*Vec4(point(0),point(1),point(2),1.0)).block<3,1>(0,0);;
        objects[categoryNo][objectNo].getPartsIds(pointGlob, ids);
    }
    else{
        std::vector<int> notUsed(3,-1);
        ids.insert(std::end(ids), std::begin(notUsed), std::end(notUsed));
    }
}

/// get realisations ids
void HOP3DBham::getRealisationsIds(int categoryNo, int objectNo, int imageNo, int u, int v, double depth, std::vector<int>& ids) const{
    ids.clear();
    ViewDependentPart lastVDpart;
    for (int overlapNo=0;overlapNo<3;overlapNo++){
        std::vector<int> idsTmp;
        imageFilterer->getRealisationsIds(overlapNo, categoryNo, objectNo, imageNo, u, v, depth, idsTmp, lastVDpart);
        ids.insert(ids.end(),idsTmp.begin(),idsTmp.end());
    }
    /// view independent ids
    /*Mat34 cameraPose(dataset->getCameraPose(categoryNo, objectNo, imageNo));
    Vec3 point(Vec3(NAN,NAN,NAN));
    imageFilterer->getPoint(categoryNo, objectNo, imageNo, u, v, point);
    if ((!std::isnan(point(0)))&&(!std::isnan(point(1)))){
        point = (cameraPose*Vec4(point(0),point(1),point(2),1.0)).block<3,1>(0,0);
        objects[categoryNo][objectNo].getPartsIds(point, ids);
    }
    else{
        std::vector<int> notUsed(3,-1);
        ids.insert(std::end(ids), std::begin(notUsed), std::end(notUsed));
    }*/
}

/// get points realisation for the cloud
void HOP3DBham::getPointsModels(int overlapNo, int categoryNo, int objectNo, int imageNo, hop3d::PartsCloud& cloudParts) const{
    cv::Mat depthImage;
    dataset->getDepthImage((int)categoryNo,(int)objectNo,(int)imageNo, depthImage);
    PointCloudUV cloudUV;
    imageFilterer->getCloud(depthImage,cloudUV);
    for (const auto &pointuv : cloudUV){
        std::vector<int> ids;
        getPartsIds(overlapNo, categoryNo, objectNo, imageNo, pointuv.u, pointuv.v, pointuv.position(2), ids);
        PointPart pointPart;
        pointPart.position=pointuv.position;
        for (int i=0;i<(int)ids.size();i++){
            pointPart.partsIds.push_back(std::make_pair(i,ids[i]));
        }
        int idsSize = (int)ids.size();
        ids.clear();
        getPartsIds(categoryNo, objectNo, imageNo, pointuv.position, ids);
        for (int i=0;i<(int)ids.size();i++){
            pointPart.partsIds.push_back(std::make_pair(i+idsSize,ids[i]));
        }
        cloudParts.push_back(pointPart);
    }
}

/// create part-coloured point clouds
void HOP3DBham::createPartClouds(){
    /// clouds for the first layer
    int layersNo=(int)hierarchy.get()->viewDependentLayers.size()+(int)hierarchy.get()->viewIndependentLayers.size();
    std::vector<std::vector<std::array<double,4>>> colors(layersNo);
    colors[0].resize(hierarchy.get()->firstLayer.size());
    std::uniform_real_distribution<double> uniformDist(0.0,1.0);
    std::default_random_engine engine;
    for (int layerNo=0;layerNo<(int)hierarchy.get()->viewDependentLayers.size();layerNo++){
        colors[layerNo+1].resize(hierarchy.get()->viewDependentLayers[layerNo].size());
    }
    for (int layerNo=0;layerNo<(int)hierarchy.get()->viewIndependentLayers.size()-1;layerNo++){
        colors[layerNo+(int)hierarchy.get()->viewDependentLayers.size()+1].resize(hierarchy.get()->viewIndependentLayers[layerNo].size());
    }
    for (size_t layerNo=0;layerNo<colors.size();layerNo++){
        for (size_t partNo=0;partNo<colors[layerNo].size();partNo++){
            std::array<double,4> color = {uniformDist(engine), uniformDist(engine), uniformDist(engine), 1.0};
            colors[layerNo][partNo] = color;
        }
    }
    /// point clouds (index: overlapNo->layerNo->objectNo)
    int overlapsNo=4;
    std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>> cloudsObj(overlapsNo, std::vector<std::vector<hop3d::PointCloudRGBA>>(layersNo)); // vector of coloured objects
    for (int overlapNo=0; overlapNo<overlapsNo; overlapNo++){
        for (size_t categoryNo=0;categoryNo<datasetInfo.categories.size();categoryNo++){
            for (size_t objectNo=0;objectNo<datasetInfo.categories[categoryNo].objects.size();objectNo++){
                std::vector<hop3d::PointCloudRGBA> objsTmp(layersNo);
                for (size_t imageNo=0;imageNo<datasetInfo.categories[categoryNo].objects[objectNo].images.size();imageNo++){
                    hop3d::PartsCloud cloudParts;
                    if (overlapNo<3){
                        getPointsModels(overlapNo, (int)categoryNo, (int)objectNo, (int)imageNo, cloudParts);
                        Mat34 cameraPose(dataset->getCameraPose((int)categoryNo, (int)objectNo, (int)imageNo));
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
                        for (int layNo=0;layNo<3;layNo++){
                            for (int pointNo=0;pointNo<(int)cloudsObj[0][layNo][objectNo].size();pointNo++){
                                PointColor pointRGBA(cloudsObj[0][layNo][objectNo][pointNo].position,std::array<double,4>({0.0,0.0,0.0,1.0}));
                                int colorsNo=0;
                                for (int overNo=0;overNo<3;overNo++){
                                    if (cloudsObj[overNo][layNo][objectNo][pointNo].color[0]>0||cloudsObj[overNo][layNo][objectNo][pointNo].color[1]>0||cloudsObj[overNo][layNo][objectNo][pointNo].color[2]>0){
                                        colorsNo++;
                                        for (int colorId=0;colorId<3;colorId++)
                                            pointRGBA.color[colorId]+=cloudsObj[overNo][layNo][objectNo][pointNo].color[colorId];
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
    notify(cloudsObj);
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
