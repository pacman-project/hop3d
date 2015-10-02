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

            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("pixelSize", &pixelSize);
            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("filterDepthScale", &filterDepthScale);

            int layersNo=6;
            partDist.resize(layersNo);
            posZ.resize(layersNo);
            for (int i=0;i<layersNo;i++){
                std::string layerName = "layer" + std::to_string (i+1);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("dist", &partDist[i]);
                model->FirstChildElement( layerName.c_str() )->QueryDoubleAttribute("posZ", &posZ[i]);
            }

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
        }
        public:
        /// Background color
        QColor backgroundColor;
        /// draw point clouds
        bool drawPointClouds;
        /// point size
        double cloudPointSize;
        /// hierarchy pixel size in patch
        double pixelSize;
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

private:
    Config config;

    /// Hierarchy to draw
    std::unique_ptr<hop3d::Hierarchy> hierarchy;

    /// mtex to lock hierarchy
    std::mutex mtxHierarchy;

    /// updateHierarchy
    bool updateHierarchyFlag;

    /// cloud list
    std::vector< std::vector<GLuint> > cloudsListLayers;

    /// layer2layer list
    std::vector< std::vector< GLuint > > linksLists;

    /// clusters list
    std::vector< std::vector< GLuint > > clustersList;

    /// background list
    std::vector< GLuint > backgroundList;

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

    /// Draw point clouds
    void drawPointClouds(void);

    /// Draw clusters
    void drawClusters(void);

    /// Create point cloud List
    GLuint createCloudList(hop3d::PointCloud& pointCloud);

    /// Create background List
    GLuint createBackgroundList(int layerNo);

    /// Create point cloud List
    GLuint createPartList(hop3d::ViewDependentPart& part, int layerNo);

    /// Create clusters List
    GLuint createClustersList(hop3d::ViewDependentPart& part, int layerNo);

    /// Create layer 2 layer List
    GLuint createLinksList(int destLayerNo);

    /// Draw layer 2 layer links
    void drawLayer2Layer(void);

    /// Draw ellipsoid
    void drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const;

    /// Draw ellipsoid
    void drawEllipsoid(const hop3d::Vec3& pos, const hop3d::Mat33& covariance) const;
};

#endif // QVISUALIZER_H_INCLUDED
