/** @file HOP3DBham.h
 *
 * implementation - Hop3d interface
 *
 */

#ifndef HOP3DBHAM_H_INCLUDED
#define HOP3DBHAM_H_INCLUDED

#include "HOP3D.h"
#include "../../external/tinyXML/tinyxml2.h"
#include "StatisticsBuilder/unbiasedStatsBuilder.h"
#include "Data/Graph.h"
#include "PartSelector/partSelectorMean.h"
#include "ImageFilter/depthImageFilter.h"
#include "ObjectComposition/objectCompositionOctree.h"
#ifdef QVisualizerBuild
#include "QVisualizer/Qvisualizer.h"
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
            /// object composition config filename
            std::string compositionConfig;
    };

private:
    /// Configuration of the module
    Config config;

    /// Statistics builder
    StatsBuilder *statsBuilder;

    /// Part selector
    PartSelector *partSelector;

    /// Image Filterer
    ImageFilter *imageFilterer;

    ///structure to store hierarchy
    std::unique_ptr<Hierarchy> hierarchy;

    ///vector of objects compositions
    std::vector<ObjectComposition *> objects;
};
}
#endif // HOP3DBHAM_H_INCLUDED
