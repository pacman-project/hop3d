#ifndef HOP3DMATH_H
#define HOP3DMATH_H
#include <iostream>
#include <memory>
#include <array>

#include "../../external/eigen/eigen3.h"

namespace hop3d {
    void transform(hop3d::Vec6& vec, const hop3d::Mat34& transform);
}
#endif /* HOP3DMATH_H */

