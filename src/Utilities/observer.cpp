//CPP File
#include "../include/Utilities/observer.h"
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
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(hierarchy);
        }
    }
}

void Subject::notify(std::vector<hop3d::ViewIndependentPart>& objectParts){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update(objectParts);
        }
    }
}

void Subject::notify3Dmodels(){
    for(vector<Observer*>::const_iterator iter = list.begin(); iter != list.end(); ++iter)
    {
        if(*iter != 0) {
            (*iter)->update3Dmodels();
        }
    }
}
