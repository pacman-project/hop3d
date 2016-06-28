
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "hop3d/Data/Defs.h"
#include "hop3d/Data/Graph.h"
#include "hop3d/Data/Cloud.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(hop3d::Hierarchy& hierarchy) = 0;
    virtual void update(const std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId, bool inference) = 0;
    virtual void update3Dmodels(void) = 0;
    virtual void updateVD(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& clouds, bool inference) = 0;
    virtual void updateVolumetric(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& clouds, bool inference) = 0;
    /// update object from filters
    virtual void update(std::vector<std::pair<int, hop3d::Mat34>>& partsPoses, int objectNo, int layerNo, bool inference) = 0;
    virtual void updateSecondLayerPart(const hop3d::PointsSecondLayer& part) = 0;
    virtual void createPartObjects() = 0;
    virtual void update(const std::vector<hop3d::Mat34>& coords) = 0;
};

class Subject
{
    //Lets keep a track of all observed objects
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(hop3d::Hierarchy& hierarchy);
    void notify(std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId, bool inference);
    void notify3Dmodels();
    void notifyVD(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& clouds, bool inference);
    void notifyVolumetric(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& clouds, bool inference);
    /// update object from filters
    void notify(std::vector<std::pair<int, hop3d::Mat34>>& partsPoses, int objectNo, int layerNo, bool inference);
    void notify(const hop3d::PointsSecondLayer& points);
    void notify(const std::vector<hop3d::Mat34>& coords);
    void createPartObjects();
};

#endif // OBSERVER_H_
