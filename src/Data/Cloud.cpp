#include "hop3d/Data/Cloud.h"

/// Euclidean distance
double hop3d::PointNormal::euclideanDistance(const hop3d::Vec3& vec) const{
    return (sqrt(pow(vec(0)-position(0),2.0)+pow(vec(1)-position(1),2.0)+pow(vec(2)-position(2),2.0)));
}

// Insertion operator
std::ostream& hop3d::operator<<(std::ostream& os, const hop3d::PointNormal& point){
    os << point.position << " " << point.normal;
    os << "\n";
    return os;
}

// Extraction operator
std::istream& hop3d::operator>>(std::istream& is, hop3d::PointNormal& point){
    is >> point.position >> point.normal;
    return is;
}
