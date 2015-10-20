
#ifndef OBSERVER_H_
#define OBSERVER_H_

#include "Data/Defs.h"
#include "Data/Graph.h"
#include <vector>
#include <list>

class Observer
{
public:
    virtual void update(hop3d::Hierarchy& hierarchy) = 0;
    virtual void update(const std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId) = 0;
    virtual void update3Dmodels(void) = 0;
};

class Subject
{
    //Lets keep a track of all observed objects
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(hop3d::Hierarchy& hierarchy);
    void notify(std::vector<hop3d::ViewIndependentPart>& objectParts, int objLayerId);
    void notify3Dmodels();
};

#endif // OBSERVER_H_
