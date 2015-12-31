/** @file HOP3D.h
 *
 * HOP3D interface
 *
 */

#ifndef _HOP3D_H_
#define _HOP3D_H_

#include "../Data/Defs.h"
#include "../Data/Vocabulary.h"
#include "Utilities/observer.h"

namespace hop3d {

/// Statistics builder interface
class HOP3D : public Subject{
public:

    /// HOP3d type
    enum Type {
        /// HOP3D Bham implementation
        HOP3D_BHAM,
    };

    /// overloaded constructor
    HOP3D(const std::string _name, Type _type) :
            name(_name), type(_type) {
    }

    /// Name of the hop3d
    virtual const std::string& getName() const {return name;}

    /// learning from the dataset
    virtual void learn(void) = 0;

    /// load hierarchy from the file
    virtual void load(std::string filename) = 0;

    /// get set of ids from hierarchy for the given input point
    virtual void getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const = 0;

    /// get training dataset info
    virtual void getDatasetInfo(hop3d::DatasetInfo& dataset) const = 0;

    /// get cloud from dataset
    virtual void getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud) const = 0;

    /// get number of points in the point cloud
    virtual size_t getNumOfPoints(int categoryNo, int objectNo, int imageNo) const = 0;

    /// get point from the point cloud
    virtual void getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const = 0;

    /// get camera pose
    virtual void getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose) const = 0;

    /// Virtual descrutor
    virtual ~HOP3D() {
    }

protected:
    /// hop3d name
    const std::string name;

    /// Map type
    Type type;
};
}

#endif // _HOP3D_H_
