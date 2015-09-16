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
            model->FirstChildElement( "hierarchy" )->QueryDoubleAttribute("layer1dist", &layer1dist);
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
        /// distance between filters
        double layer1dist;
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
    std::vector<GLuint> cloudsList;

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

    /// Create point cloud List
    GLuint createCloudList(hop3d::PointCloud& pointCloud);

    /// Draw ellipsoid
    void drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const;

    /// Draw ellipsoid
    void drawEllipsoid(const hop3d::Vec3& pos, const hop3d::Mat33& covariance) const;
};

#endif // QVISUALIZER_H_INCLUDED
