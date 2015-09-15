#ifndef DATA_DEFS_H
#define DATA_DEFS_H
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>
#include "opencv2/core/core.hpp"

namespace hop3d {

//read: http://www-01.ibm.com/support/knowledgecenter/SSLTBW_2.1.0/com.ibm.zos.v2r1.cbcpx01/datatypesize64.htm
//read: http://sourceforge.net/p/predef/wiki/Home/
#ifdef _WIN32
// Windows 32/64-bit, ILP32/LLP64
typedef signed char				I8;
typedef signed short			I16;
typedef signed int				I32;
typedef __int64					I64;

typedef unsigned char			U8;
typedef unsigned short			U16;
typedef unsigned int			U32;
typedef unsigned __int64		U64;

typedef double					F32;
typedef double					F64;

#elif __linux__
// Linux 32/64-bit, ILP32/LP64
typedef signed char				I8;
typedef signed short			I16;
typedef signed int				I32;
#ifdef __amd64
typedef long int				I64;
#else
typedef long long				I64;
#endif

typedef unsigned char			U8;
typedef unsigned short			U16;
typedef unsigned int			U32;
#ifdef __amd64
typedef unsigned long int 		U64;
#else
typedef unsigned long long		U64;
#endif

typedef double					F32;
typedef double					F64;
#endif

/// 2 element vector class
typedef Eigen::Vector2d Vec2;
/// 3 element vector class
typedef Eigen::Vector3d Vec3;
/// Matrix representation of SO(2) group of rotations or other 2D matrices
typedef Eigen::Matrix<double, 2, 2> Mat22;
/// Matrix representation of SO(3) group of rotations or other 3D matrices
typedef Eigen::Matrix<double, 3, 3> Mat33;
/// Homogeneous representation of SE(3) rigid body transformations
typedef Eigen::Transform<double, 3, Eigen::Affine> Mat34;

/// Depth image filter representation
class Filter {
public:
    /// set of filters
    typedef std::vector<Filter> Seq;

    /// id of the filter
    int id;
    /// mask of the filter
    cv::Mat mask;
    /// surface of the filter
    cv::Mat patch;
    /// vector normal to the center of the patch
    Vec3 normal;

    /// compute distance between filters
    static double distance(const Filter& filterA, const Filter& filterB);

    ///Construction
    Filter(){};
};

/// 2D image coordinates
class ImageCoords {
public:
    /// set of features
    typedef std::vector<ImageCoords> Seq;

    /// 2D feature location
    double u;
    double v;

    /// Construction
    ImageCoords(){};

    /// Construction
    ImageCoords(double _u, double _v) : u(_u), v(_v) {};
};

class ImageCoordsDepth : public ImageCoords {
public:
    /// set of features
    typedef std::vector<ImageCoords> Seq;

    ///Depth value
    double depth;

    /// Construction
    ImageCoordsDepth(){};

    /// Construction
    ImageCoordsDepth(double _u, double _v, double _depth) : ImageCoords(_u, _v), depth(_depth) {};
};

/// Octet representation
class Octet {
public:
    /// set of octets
    typedef std::vector<Octet> Seq;

    /// id of the filter
    std::array<std::array<int,3>,3> filterIds;
    /// filter response
    std::array<std::array<double,3>,3> responses;
    /// mask of the filter
    std::array<std::array<ImageCoordsDepth,3>,3> filterPos;
    /// camera pose Id
    int poseId;

    /// Construction
    Octet(){};

    /// compute distance between octets -- dot product for normals for each filter
    static double distance(const Octet& octetA, const Octet& octetB, const Filter::Seq& filters);

    /// Print
    void print() const;
};

/// 2D Gaussian
class Gaussian3D{
public:
    /// set of 2d Gaussians
    typedef std::vector<Gaussian3D> Seq;

    /// position
    Vec3 mean;
    /// covariance matrix
    Mat33 covariance;

    /// Construction
    Gaussian3D(){};
};

}
#endif /* DATA_DEFS_H */

