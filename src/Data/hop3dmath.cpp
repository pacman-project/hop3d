#include "hop3d/Data/Defs.h"
#include <chrono>
#include <random>

namespace hop3d {
    void transform(hop3d::Vec6& vec, const hop3d::Mat34& transform){
        Vec4 pos(vec(0),vec(1),vec(2),1);
        pos = transform*pos;
        Vec3 rot = transform.rotation()*vec.block<3,1>(3,0);
        for (int i = 0;i<3;i++){
            vec(i)=pos(i);
            vec(i+3)=rot(i);
        }
    }
}
