/** @file QVisualizer.h
 *
 * implementation - QGLVisualizer
 *
 */

#ifndef QVISUALIZER_H_INCLUDED
#define QVISUALIZER_H_INCLUDED

#include "Data/Defs.h"
#include "Data/Cloud.h"
#include "../include/Utilities/observer.h"
#include "../external/tinyXML/tinyxml2.h"
#include <QGLViewer/qglviewer.h>
#include <thread>
#include <mutex>
#include <iostream>

/// Map implementation
class QGLVisualizer: public QGLViewer, public Observer{
public:
    /// Pointer
    typedef std::unique_ptr<QGLVisualizer> Ptr;
    /// 3D object
    typedef std::vector<hop3d::ViewIndependentPart> Object3D;
    /// vector of 3D objects
    typedef std::vector<Object3D> Object3DSeq;

    /// filter 3D position
    class Filter3D{
    public:
        /// id of the filter
        int filterId;
        /// position of the filter
        hop3d::Mat34 filterPose;
    };

    class Config{
      public:
        Config() {
        }
        Config(std::string configFilename){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load Visualizer config file.\n";
            tinyxml2::XMLElement * model = config.FirstChildElement( "VisualizerConfig" );
            model->FirstChildElement( "parameters" )->QueryIntAttribute("verbose", &verbose);
            double rgba[4]={0,0,0,0};
            model->FirstChildElement( "background" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "background" )->QueryDoubleAttribute("alpha", &rgba[3]);
            backgroundColor.setRedF(rgba[0]); backgroundColor.setGreenF(rgba[1]);
            backgroundColor.setBlueF(rgba[2]); backgroundColor.setAlphaF(rgba[3]);
            model->FirstChildElement( "pointCloud" )->QueryBoolAttribute("drawPointClouds", &drawPointClouds);
            model->FirstChildElement( "pointCloud" )->QueryDoubleAttribute("cloudPointSize", &cloudPointSize);
            model->FirstChildElement( "pointCloud" )->QueryBoolAttribute("useNormalCloud", &useNormalCloud);

            model->FirstChildElement( "surface" )->QueryBoolAttribute("drawSurfaces", &drawSurfaces);
            model->FirstChildElement( "surface" )->QueryBoolAttribute("useNormalSurf", &useNormalSurf);

            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("pixelSize", &pixelSize);
            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("filterDepthScale", &filterDepthScale);
            model->FirstChildElement( "hierarchy" )->QueryBoolAttribute("draw3Dobjects", &draw3Dobjects);
            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("voxelSize", &voxelSize);

            int layersNo=7;
            partDist.resize(layersNo);
            posZ.resize(layersNo);
            for (int i=0;i<layersNo;i++){
                std::string layerName = "layer" + std::to_string (i+1);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("dist", &partDist[i]);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("posZ", &posZ[i]);
            }

            layersNo=6;
            objectsDraw.resize(layersNo);
            objectsDist.resize(layersNo);
            objectsPosY.resize(layersNo);
            objectsPosZ.resize(layersNo);
            for (int i=0;i<layersNo;i++){
                std::string layerName = "objectsLayer" + std::to_string (i+1);
                bool drawObj;
                model->FirstChildElement( layerName.c_str() )->QueryBoolAttribute("draw", &drawObj);
                objectsDraw[i] = drawObj;
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("dist", &objectsDist[i]);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("posY", &objectsPosY[i]);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("posZ", &objectsPosZ[i]);
            }

            model->FirstChildElement( "objectsParts" )->QueryBoolAttribute("drawPartObjects", &drawPartObjects);
            partObjectsPos.resize(3);
            model->FirstChildElement( "objectsParts" )->QueryDoubleAttribute("dist", &partObjectsPos[0]);
            model->FirstChildElement( "objectsParts" )->QueryDoubleAttribute("posY", &partObjectsPos[1]);
            model->FirstChildElement( "objectsParts" )->QueryDoubleAttribute("posZ", &partObjectsPos[2]);

            model->FirstChildElement( "layer2layer" )->QueryBoolAttribute("drawLayer2Layer", &drawLayer2Layer);
            model->FirstChildElement( "layer2layer" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "layer2layer" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "layer2layer" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "layer2layer" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "layer2layer" )->QueryDoubleAttribute("width", &layer2LayerWidth);
            layer2LayerColor.setRedF(rgba[0]); layer2LayerColor.setGreenF(rgba[1]);
            layer2LayerColor.setBlueF(rgba[2]); layer2LayerColor.setAlphaF(rgba[3]);

            model->FirstChildElement( "clusters" )->QueryBoolAttribute("drawClusters", &drawClusters);
            model->FirstChildElement( "clusters" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "clusters" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "clusters" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "clusters" )->QueryDoubleAttribute("alpha", &rgba[3]);
            clustersColor.setRedF(rgba[0]); clustersColor.setGreenF(rgba[1]);
            clustersColor.setBlueF(rgba[2]); clustersColor.setAlphaF(rgba[3]);

            model->FirstChildElement( "normals" )->QueryBoolAttribute("drawNormals", &drawNormals);
            model->FirstChildElement( "normals" )->QueryDoubleAttribute("red", &rgba[0]);
            model->FirstChildElement( "normals" )->QueryDoubleAttribute("green", &rgba[1]);
            model->FirstChildElement( "normals" )->QueryDoubleAttribute("blue", &rgba[2]);
            model->FirstChildElement( "normals" )->QueryDoubleAttribute("alpha", &rgba[3]);
            model->FirstChildElement( "normals" )->QueryDoubleAttribute("scale", &normalsScale);
            normalsColor.setRedF(rgba[0]); normalsColor.setGreenF(rgba[1]);
            normalsColor.setBlueF(rgba[2]); normalsColor.setAlphaF(rgba[3]);
        }
        public:
        /// Background color
        QColor backgroundColor;
        /// draw point clouds
        bool drawPointClouds;
        /// draw point clouds
        bool useNormalCloud;
        /// draw surfaces
        bool drawSurfaces;
        /// use normal to the surface
        bool useNormalSurf;
        /// point size
        double cloudPointSize;
        /// hierarchy pixel size in patch
        double pixelSize;
        /// draw 3D objects
        bool draw3Dobjects;
        ///voxel size
        double voxelSize;
        /// distance between parts in i-th layers
        std::vector<double> partDist;
        /// "z" coordinate of the i-t hierarchy layer
        std::vector<double> posZ;
        /// Draw layer 2 layer link
        bool drawLayer2Layer;
        /// link color color
        QColor layer2LayerColor;
        /// layer2layer link width
        double layer2LayerWidth;
        /// Draw clusters
        bool drawClusters;
        /// Cluster color
        QColor clustersColor;
        /// verbose
        int verbose;
        /// scale depth of filter patches
        double filterDepthScale;
        /// draw normals
        bool drawNormals;
        /// normals color
        QColor normalsColor;
        /// scale normal vector
        double normalsScale;
        /// draw flags for objects
        std::vector<bool> objectsDraw;
        /// distance between objects in i-th layers (x coordinate)
        std::vector<double> objectsDist;
        /// "z" coordinate of the objects at i-th hierarchy layer
        std::vector<double> objectsPosZ;
        /// "y" coordinate of the objects at i-th hierarchy layer
        std::vector<double> objectsPosY;

        /// draw objects colored by part id
        bool drawPartObjects;
        /// distance parameters
        std::vector<double> partObjectsPos;

    };

    /// Construction
    QGLVisualizer(void);

    /// Construction
    QGLVisualizer(std::string configFile);

    /// Construction
    QGLVisualizer(Config _config);

    /// Destruction
    ~QGLVisualizer(void);

    /// Observer update
    void update(hop3d::Hierarchy& _hierarchy);

    /// Update 3D object model
    void update(const std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId);

    /// Update 3D object models
    void update3Dmodels(void);

    /// update part clouds
    void update(std::vector<std::vector<hop3d::PointCloudRGBA>>& objects);

    /// update object from filters
    void update(std::vector<std::pair<int, hop3d::Mat34>>& filtersPoses, int objectNo);

private:
    Config config;

    /// Hierarchy to draw
    std::unique_ptr<hop3d::Hierarchy> hierarchy;

    /// mtex to lock hierarchy
    std::mutex mtxHierarchy;

    /// updateHierarchy
    bool updateHierarchyFlag;

    /// update 3D models
    bool update3DModelsFlag;

    /// updatePartsObjectsFlag
    bool updatePartsObjectsFlag;

    /// cloud list
    std::vector< std::vector<GLuint> > cloudsListLayers;

    /// layer2layer list
    std::vector< std::vector< GLuint > > linksLists;

    /// clusters list
    std::vector< std::vector< GLuint > > clustersList;

    /// background list
    std::vector< GLuint > backgroundList;

    /// objects indexed by layerNo
    std::vector<Object3DSeq> layersOfObjects;

    /// object clouds coloured by part id
    std::vector<std::vector<hop3d::PointCloudRGBA>> partClouds;

    /// clusters list
    std::vector< std::vector< GLuint > > objects3Dlist;

    /// objects drawn from filter poses
    std::vector<std::vector<Filter3D>> objectsFromFilters;

    /// partObjects list
    std::vector< GLuint > partObjectsLists;

    /// active layer for partObjects
    int activeLayer;

    /// draw objects
    void draw();

    /// draw objects
    void animate();

    /// initialize visualizer
    void init();

    /// generate help string
    std::string help() const;

    ///update hierarchy
    void updateHierarchy();

    /// Update 3D object models
    void update3Dobjects(void);

    /// Draw point clouds
    void drawPointClouds(void);

    /// Update parts objects
    void updatePartsObjects(void);

    /// create partObjects
    void createPartObjects();

    /// Draw clusters
    void drawClusters(void);

    /// Draw objects
    void draw3Dobjects(void);

    /// Draw part objects
    void drawPartObjects(void);

    /// Create point cloud List
    GLuint createPartObjList(const std::vector<hop3d::PointCloudRGBA>& objects);

    /// Create point cloud List
    GLuint createCloudList(hop3d::PointCloud& pointCloud, hop3d::Vec3& normal);

    /// Create background List
    GLuint createBackgroundList(int layerNo);

    /// Create view independent part list
    GLuint createPartList(hop3d::ViewDependentPart& part, int layerNo);

    /// Create point cloud List
    GLuint createVIPartList(hop3d::ViewIndependentPart& part, int layerNo);

    /// Create objects lists
    GLuint createObjList(const std::vector<hop3d::ViewIndependentPart>& parts, int layerNo);

    /// Create point cloud List from filters (planar patches)
    GLuint createObjList(const std::vector<Filter3D>& filters);

    /// Create clusters List
    GLuint createClustersList(hop3d::ViewDependentPart& part, int layerNo);

    /// Create clusters List
    GLuint createVIClustersList(hop3d::ViewIndependentPart& part, int layerNo);

    /// Create layer 2 layer List
    GLuint createLinksList(int destLayerNo);

    /// Create layer 2 layer List (View independent part)
    GLuint createVILinksList(int destLayerNo);

    /// transpose ids matrix
    void transposeIds(std::array<std::array<int,3>,3>& ids);

    /// transpose gaussians matrix
    void transposeGaussians(std::array<std::array<hop3d::Gaussian3D,3>,3>& gaussians);

    /// flip horizontal ids matrix
    void flipIds(std::array<std::array<int,3>,3>& ids);

    /// flip horizontal gaussians matrix
    void flipGaussians(std::array<std::array<hop3d::Gaussian3D,3>,3>& ids);

    /// Draw layer 2 layer links
    void drawLayer2Layer(void);

    /// keyboard events
    void keyPressEvent(QKeyEvent *e);

    /// Draw ellipsoid
    void drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const;

    /// Draw ellipsoid
    void drawEllipsoid(const hop3d::Vec3& pos, const hop3d::Mat33& covariance) const;
};

#endif // QVISUALIZER_H_INCLUDED
