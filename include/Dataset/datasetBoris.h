/** @file datasetoris.h
 *
 * implementation - dataset Boris
 *
 */

#ifndef DATASET_BORIS_H_INCLUDED
#define DATASET_BORIS_H_INCLUDED

#include "dataset.h"
#include "../../external/tinyXML/tinyxml2.h"
#include "Utilities/depthSensorModel.h"

namespace hop3d {
    /// create a single dataset object
    Dataset* createBorisDataset(void);
    /// create a single dataset object
    Dataset* createBorisDataset(std::string config, std::string sensorConfig);


/// Dataset implementation
class BorisDataset: public Dataset {
public:
    /// Pointer
    typedef std::unique_ptr<BorisDataset> Ptr;

    /// Construction
    BorisDataset(void);

    /// Construction
    BorisDataset(std::string config, std::string sensorConfig);

    /// get image from dataset
    void getDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const;

    /// get dataset info
    void getDatasetInfo(hop3d::DatasetInfo& dataset) const;

    /// Destruction
    ~BorisDataset(void);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            // Dataset structure
            DatasetInfo dataset;
            // verbose
            int verbose;
    };

private:
    ///Configuration of the module
    Config config;

    /// sensor model
    DepthSensorModel sensorModel;
};

}
#endif // DATASET_BORIS_H_INCLUDED
