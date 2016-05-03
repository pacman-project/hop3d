/** @file HOP3DBham.h
 *
 * implementation - Hop3d interface
 *
 */

#ifndef HOP3DBHAM_H_INCLUDED
#define HOP3DBHAM_H_INCLUDED

#include "HOP3D.h"
#include "tinyXML/tinyxml2.h"
#include "hop3d/StatisticsBuilder/unbiasedStatsBuilder.h"
#include "hop3d/Data/Graph.h"
#include "hop3d/PartSelector/partSelectorMean.h"
#include "hop3d/PartSelector/partSelectorAgglomerative.h"
#include "hop3d/ImageFilter/depthImageFilter.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include "hop3d/Dataset/datasetBoris.h"
#include "hop3d/Dataset/datasetPacman.h"
#include "hop3d/Utilities/depthSensorModel.h"
#include "hop3d/ObjectComposition/objectCompositionOctree.h"
#ifdef QVisualizerBuild
#include "hop3d/QVisualizer/Qvisualizer.h"
#endif

namespace hop3d {
    /// create a single hop3d module
    HOP3D* createHOP3DBham(void);
    /// create a single hop3d module
    HOP3D* createHOP3DBham(std::string config);


/// Unbiased statistics implementation
class HOP3DBham: public HOP3D{
public:
    /// Pointer
    typedef std::unique_ptr<HOP3DBham> Ptr;

    /// Construction
    HOP3DBham(void);

    /// Construction
    HOP3DBham(std::string config);

    /// learining from the dataset
    void learn(void);

    /// load hierarchy from the file
    void load(std::string filename);

    /// load inference results from the file
    void loadInference(std::string filename);

    /// inference
    void inference(void);

    /// inference
    void inference(std::vector<std::pair<cv::Mat, Mat34>>& cameraFrames, int categoryNo, int objectNo);

    /// inference
    void inference(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& images);

    /// get set of ids from hierarchy for the given input point
    //void getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const;

    /// get training dataset info
    void getDatasetInfo(hop3d::DatasetInfo& _dataset, bool inference) const;

    /// returns paths for each cloud/image
    void getCloudPaths(std::vector<std::string>& paths, bool inference) const;

    /// get cloud from dataset
    void getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud, bool inference) const;

    /// get cloud from dataset
    void getCloud(const std::string& path, hop3d::PointCloud& cloud, bool inference) const;

    /// get hierarchy graph
    void getHierarchy(Hierarchy::IndexSeqMap& hierarchyGraph) const;

    /// get parts realization
    void getPartsRealisation(const std::string& path, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const;

    /// get parts realization
    void getPartsRealisation(int categoryNo, int objectNo, int imageNo, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const;

    /// get parts realization
    void getPartsRealisationCloud(int categoryNo, int objectNo, int imageNo, PartsClouds& parts, bool inference) const;

    /// get parts realization
    void getPartsRealisationCloud(const std::string& path, PartsClouds& parts, bool inference) const;

    /// get maps from point to part realisation
    void getCloud2PartsMap(const std::string& path, Hierarchy::IndexSeqMap& points2parts, bool inference) const;

    /// get maps from point to part realisation
    void getCloud2PartsMap(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSeqMap& points2parts, bool inference) const;

    /// get number of points in the point cloud
    size_t getNumOfPoints(int categoryNo, int objectNo, int imageNo, bool inference) const;

    /// get point from the point cloud
    //void getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const;

    /// get camera pose
    void getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose, bool inference) const;

    /// get camera pose
    void getSensorFrame(const std::string& path, Mat34& cameraPose, bool inference) const;

    /// get realisations graph
    void getRealisationsGraph(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSetMap& realisationsGraph, bool inference) const;

    /// get realisations graph
    void getRealisationsGraph(const std::string& path, Hierarchy::IndexSetMap& realisationsGraph, bool inference) const;

    #ifdef QVisualizerBuild
        ///Attach visualizer
        void attachVisualizer(QGLVisualizer* visualizer);
    #endif

    /// Destruction
    ~HOP3DBham(void);

    class Config{
      public:
        Config() : verbose(0){
        }

        Config(std::string configFilename);
        public:
            /// Verbose
            int verbose;
            /// No of view dependent layers
            int viewDependentLayersNo;
            /// No of view independent layers
            int viewIndependentLayersNo;
            /// statistics builder config filename
            std::string statsConfig;
            /// part selector config filename
            std::string selectorConfig;
            /// part selector type
            int partSelectorType;
            /// image filterer config filename
            std::string filtererConfig;
            /// filter type
            int filterType;
            /// object composition config filename
            std::string compositionConfig;
            /// camera config filename
            std::string cameraConfig;
            /// dataset config filename (training)
            std::string datasetConfig;
            /// dataset config filename (testing)
            std::string configFilenameTest;
            /// dataset type
            int datasetType;
            /// save hierarchy to file
            bool save2file;
            /// save inference results to file
            bool saveInference;
            /// inference to layer
            int inferenceUpToLayer;
            /// inference results filename
            std::string filename2saveInference;
            /// use visualization
            bool useVisualization;
            /// hierarchy filename
            std::string filename2save;
    };

private:
    /// get set of ids from hierarchy for the given input point (view-dependent layers)
    void getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, int u, int v, double depth, std::vector<int>& ids, bool inference) const;

    /// get set of ids from hierarchy for the given input point (view-independent layers)
    void getPartsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, const Vec3& point, std::vector<int>& ids, bool inference) const;

    /// get set of ids from hierarchy for the given input point
    void getRealisationsIds(int overlapNo, int categoryNo, int objectNo, int imageNo, int u, int v, double depth, std::vector<int>& ids) const;

    /// create part-coloured point clouds
    void createPartClouds(bool inference);

    /// create objects from parts
    void createObjsFromParts(bool inference);

    /// get points realisation for the cloud
    void getPointsModels(int overlapNo, int categoryNo, int objectNo, int imageNo, hop3d::PartsCloud& cloudParts, bool inference) const;

    /// convert parts map to parts cloud
    void convertPartsMap2PartsCloud(const Hierarchy::IndexSeqMap& points2parts, PartsClouds& partsCloud) const;

    /// notify visualizer
    void notifyVisualizer(void);

    /// return object which are build from part id
    void getObjectsBuildFromPart(int partId, int layerNo, std::map<std::string,int>& objectNames);

    /// Configuration of the module
    Config config;

    /// Statistics builder
    StatsBuilder *statsBuilder;

    /// Part selector
    PartSelector *partSelector;

    /// Image Filterer
    ImageFilter *imageFilterer;

    /// dataset
    std::unique_ptr<Dataset> datasetTrain;

    /// dataset
    std::unique_ptr<Dataset> datasetTest;

    /// structure which stores info about dataset
    DatasetInfo datasetInfoTrain;

    /// structure which stores info about dataset
    DatasetInfo datasetInfoTest;

    ///structure to store hierarchy
    std::unique_ptr<Hierarchy> hierarchy;

    ///depth camera model
    std::unique_ptr<DepthSensorModel> depthCameraModel;

    ///vector of objects compositions
    std::vector<std::vector<ObjectCompositionOctree>> objects;

    ///vector of objects compositions
    std::vector<std::vector<ObjectCompositionOctree>> objectsInference;

    std::vector<std::vector<std::array<double,4>>> colors;

    int categoryInferenceCount;
};
}
#endif // HOP3DBHAM_H_INCLUDED
