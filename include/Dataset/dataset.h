/** @file dataset.h
 *
 * Dataset interface
 *
 */

#ifndef _DATASET_H_
#define _DATASET_H_

#include "../Data/Defs.h"

namespace hop3d {

/// Dataset interface
class Dataset {
public:

    /// Dataset type
    enum Type {
        /// Boris dataset
        DATASET_BORIS,
    };

    /// overloaded constructor
    Dataset(const std::string _name, Type _type) :
            name(_name), type(_type) {
    }

    /// Name of the dataset
    virtual const std::string& getName() const {return name;}

    /// get image from dataset
    virtual void getDepthImage(int categoryNo, int objectNo, int imageNo, cv::Mat& depthImage) const = 0;

    /// get dataset info
    virtual void getDatasetInfo(hop3d::DatasetInfo& dataset) const = 0;

    /// get camera pose
    virtual Mat34 getCameraPose(int categoryNo, int objectNo, int imageNo) const = 0;

    /// Virtual descrutor
    virtual ~Dataset() {
    }

protected:
    /// dataset name
    const std::string name;

    /// Dataset type
    Type type;
};
}

#endif // _DATASET_H_
