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
#include "hop3d/ImageFilter/depthImageFilter.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include "hop3d/Dataset/datasetBoris.h"
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

    /// get set of ids from hierarchy for the given input point
    void getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const;

    /// get training dataset info
    void getDatasetInfo(hop3d::DatasetInfo& _dataset) const;

    /// returns paths for each cloud/image
    void getCloudPaths(std::vector<std::string>& paths) const;

    /// get cloud from dataset
    void getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud) const;

    /// get cloud from dataset
    void getCloud(const std::string& path, hop3d::PointCloud& cloud) const;

    /// get hierarchy graph
    void getHierarchy(Hierarchy::IndexSeqMap& hierarchyGraph) const;

    /// get parts realization
    void getPartsRealisation(const std::string& path, std::vector<ViewIndependentPart::Part3D>& parts) const;

    /// get parts realization
    void getPartsRealisation(int categoryNo, int objectNo, int imageNo, std::vector<ViewIndependentPart::Part3D>& parts) const;

    /// get maps from point to part realisation
    void getCloud2PartsMap(const std::string& path, Hierarchy::IndexSeqMap& points2parts) const;

    /// get maps from point to part realisation
    void getCloud2PartsMap(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSeqMap& points2parts) const;

    /// get number of points in the point cloud
    size_t getNumOfPoints(int categoryNo, int objectNo, int imageNo) const;

    /// get point from the point cloud
    void getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const;

    /// get camera pose
    void getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose) const;

    /// get camera pose
    void getSensorFrame(const std::string& path, Mat34& cameraPose) const;

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
            /// image filterer config filename
            std::string filtererConfig;
            /// filter type
            int filterType;
            /// object composition config filename
            std::string compositionConfig;
            /// camera config filename
            std::string cameraConfig;
            /// dataset config filename
            std::string datasetConfig;
            /// dataset type
            int datasetType;
            /// save hierarchy to file
            bool save2file;
            /// hierarchy filename
            std::string filename2save;
    };

private:
    /// get set of ids from hierarchy for the given input point
    void getPartsIds(int categoryNo, int objectNo, int imageNo, int u, int v, std::vector<int>& ids) const;

    /// create part-coloured point clouds
    void createPartClouds();

    /// Configuration of the module
    Config config;

    /// Statistics builder
    StatsBuilder *statsBuilder;

    /// Part selector
    PartSelector *partSelector;

    /// Image Filterer
    ImageFilter *imageFilterer;

    /// dataset
    Dataset *dataset;

    /// structure which stores info about dataset
    DatasetInfo datasetInfo;

    ///structure to store hierarchy
    std::unique_ptr<Hierarchy> hierarchy;

    ///depth camera model
    std::unique_ptr<DepthSensorModel> depthCameraModel;

    ///vector of objects compositions
    std::vector<std::vector<ObjectCompositionOctree>> objects;
};
}
#endif // HOP3DBHAM_H_INCLUDED
