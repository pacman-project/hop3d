
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
};

class Subject
{
    //Lets keep a track of all the shops we have observing
    std::vector<Observer*> list;

public:
    void attach(Observer *observer);
    void detach(Observer *observer);
    void notify(hop3d::Hierarchy& hierarchy);
};

#endif // OBSERVER_H_
