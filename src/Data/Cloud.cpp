#include "Data/Cloud.h"


/// Euclidean distance
double hop3d::PointNormal::euclideanDistance(const hop3d::Vec3& vec) const{
    return (sqrt(pow(vec(0)-position(0),2.0)+pow(vec(1)-position(1),2.0)+pow(vec(2)-position(2),2.0)));
}
