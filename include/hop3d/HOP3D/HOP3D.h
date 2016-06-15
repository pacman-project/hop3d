/** @file HOP3D.h
 *
 * HOP3D interface
 *
 */

#ifndef _HOP3D_H_
#define _HOP3D_H_

#include "../Data/Defs.h"
#include "../Data/Vocabulary.h"
#include "hop3d/Utilities/observer.h"
#include <unordered_map>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace hop3d {

/// Statistics builder interface
class HOP3D : public Subject{
public:

    /// HOP3d type
    enum Type {
        /// HOP3D Bham implementation
        HOP3D_BHAM,
    };

    /// parts realisation cloud (set of points (value) which represent realisation id (key))
    typedef std::unordered_map<std::uint32_t,std::set<std::uint32_t>> PartsClouds;

    /// overloaded constructor
    HOP3D(const std::string _name, Type _type) :
            name(_name), type(_type) {
    }

    /// Name of the hop3d
    virtual const std::string& getName() const {return name;}

    /// learning from the dataset
    virtual void learn(void) = 0;

    /// learning from the dataset
    virtual void learnIncremental(void) = 0;

    /// load hierarchy from the file
    virtual void load(std::string filename) = 0;

    /// load inference results from the file
    virtual void loadInference(std::string filename) = 0;

    /// inference
    virtual void inference(void) = 0;

    /// inference
    virtual void inference(std::vector<std::pair<cv::Mat, Mat34>>& cameraFrames, std::vector<std::string>& names) = 0;

    /// inference
    virtual void inference(std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>& images) = 0;

    /// get set of ids from hierarchy for the given input point
    //virtual void getPartsIds(const std::string& path, int u, int v, std::vector<int>& ids) const = 0;

    /// get training dataset info
    virtual void getDatasetInfo(hop3d::DatasetInfo& dataset, bool inference) const = 0;

    /// returns paths for each cloud/image
    virtual void getCloudPaths(std::vector<std::string>& paths, bool inference) const = 0;

    /// get cloud from dataset
    //virtual void getCloud(int categoryNo, int objectNo, int imageNo, hop3d::PointCloud& cloud, bool inference) const = 0;

    /// get cloud from dataset
    virtual void getCloud(const std::string& path, hop3d::PointCloud& cloud, bool inference) const = 0;

    /// get hierarchy graph
    virtual void getHierarchy(Hierarchy::IndexSeqMap& hierarchyGraph) const = 0;

    /// get parts realization
    virtual void getPartsRealisation(const std::string& path, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const = 0;

    /// get parts realization
    //virtual void getPartsRealisation(int categoryNo, int objectNo, int imageNo, std::vector<ViewIndependentPart::Part3D>& parts, bool inference) const = 0;

    /// get parts realization
    //virtual void getPartsRealisationCloud(int categoryNo, int objectNo, int imageNo, PartsClouds& parts, bool inference) const = 0;

    /// get parts realization
    virtual void getPartsRealisationCloud(const std::string& path, PartsClouds& parts, bool inference) const = 0;

    /// get number of points in the point cloud
    //virtual size_t getNumOfPoints(int categoryNo, int objectNo, int imageNo, bool inference) const = 0;

    /// get point from the point cloud
    //virtual void getPoint(int categoryNo, int objectNo, int imageNo, size_t pointNo, Vec3& point) const = 0;

    /// get camera pose
    //virtual void getSensorFrame(int categoryNo, int objectNo, int imageNo, Mat34& cameraPose, bool inference) const = 0;

    /// get camera pose
    virtual void getSensorFrame(const std::string& path, Mat34& cameraPose, bool inference) const = 0;

    /// get maps from point to part realisation
    virtual void getCloud2PartsMap(const std::string& path, Hierarchy::IndexSeqMap& points2parts, bool inference) const = 0;

    /// get maps from point to part realisation
    //virtual void getCloud2PartsMap(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSeqMap& points2parts, bool inference) const = 0;

    /// get realisations graph
    //virtual void getRealisationsGraph(int categoryNo, int objectNo, int imageNo, Hierarchy::IndexSetMap& hierarchyGraph, bool inference) const = 0;

    /// get realisations graph
    virtual void getRealisationsGraph(const std::string& path, Hierarchy::IndexSetMap& realisationsGraph, bool inference) const = 0;

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
