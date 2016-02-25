//CPP File
#include "hop3d/Utilities/observer.h"
#include <algorithm>
#include <iostream>

using namespace std;

void Subject::attach(Observer *observer){
    list.push_back(observer);
}
void Subject::detach(Observer *observer){
    list.erase(std::remove(list.begin(), list.end(), observer), list.end());
}

void Subject::notify(hop3d::Hierarchy& hierarchy){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->update(hierarchy);
        }
    }
}

void Subject::notify(std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId, bool inference){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->update(objectParts, objLayerId, inference);
        }
    }
}

/// update object from filters
void Subject::notify(std::vector<std::pair<int, hop3d::Mat34>>& partsPoses, int objectNo, int layerNo, bool inference){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter) {
        if(*iter != 0) {
            (*iter)->update(partsPoses, objectNo, layerNo, inference);
        }
    }
}

/// update part second layer
void Subject::notify(const hop3d::PointsSecondLayer& points){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter) {
        if(*iter != 0) {
            (*iter)->updateSecondLayerPart(points);
        }
    }
}

void Subject::notify(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& clouds){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->update(clouds);
        }
    }
}

void Subject::notify3Dmodels(){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->update3Dmodels();
        }
    }
}

void Subject::notify(const std::vector<hop3d::Mat34>& coords){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->update(coords);
        }
    }
}

void Subject::createPartObjects(){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter){
        if(*iter != 0) {
            (*iter)->createPartObjects();
        }
    }
}
