#ifndef DATA_DEFS_H
#define DATA_DEFS_H
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>

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

typedef Eigen::Vector3d         Vec3;
typedef Eigen::Vector4i         Face;
class PointNormal{
public:
    Vec3 position;
    Vec3 normal;
};
typedef Eigen::Matrix3d         ReferenceFrame;




}
#endif /* DATA_DEFS_H */

