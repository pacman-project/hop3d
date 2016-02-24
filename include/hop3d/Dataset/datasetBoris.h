/** @file datasetoris.h
 *
 * implementation - dataset Boris
 *
 */

#ifndef DATASET_BORIS_H_INCLUDED
#define DATASET_BORIS_H_INCLUDED

#include "dataset.h"
#include "tinyXML/tinyxml2.h"
#include "hop3d/Utilities/depthSensorModel.h"
#include <string>
#include <unordered_map>

namespace hop3d {
    /// create a single dataset object
    std::unique_ptr<Dataset> createBorisDataset(void);
    /// create a single dataset object
    std::unique_ptr<Dataset> createBorisDataset(std::string config, std::string sensorConfig);


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

    /// read camera pose from file
    Mat34 getCameraPose(int categoryNo, int objectNo, int imageNo) const;

    /// translate path to filename into category, object, image numbers
    void translateString(const std::string& path, int& categoryNo, int& objectNo, int& imageNo) const;

    /// read number of point for the point cloud dataset
    size_t getNumOfPoints(int categoryNo, int objectNo, int imageNo) const;

    /// get point from the point cloud
    void getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const;

    /// Destruction
    ~BorisDataset(void);

    class Config{
      public:
        Config() {
        }

        Config(std::string configFilename);
        public:
            /// Dataset structure
            DatasetInfo dataset;
            /// verbose
            int verbose;
            /// dataset type
            std::string type;
            /// dataset path
            std::string path;
            /// categories map from category name to category id
            std::unordered_map<std::string,int> categories;
            /// categories map from object name to object id
            std::unordered_map<std::string,int> objects;
            /// categories map from image name to image id
            std::unordered_map<std::string,int> images;
    };

private:
    ///Configuration of the module
    Config config;

    /// sensor model
    DepthSensorModel sensorModel;

    /// read depth image from dataset
    void readDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const;

    /// read camera pose from file
    static Mat34 readCameraPose(std::string& filename);
};

}
#endif // DATASET_BORIS_H_INCLUDED
