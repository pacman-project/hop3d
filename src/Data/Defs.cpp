#include "Data/Defs.h"

namespace hop3d {

    /// compute distance between filters -- dot product for normals
    double Filter::distance (const Filter& filterA, const Filter& filterB){
        return (filterA.id==filterB.id) ? 0 : (double)(1.0-filterA.normal.adjoint()*filterB.normal);
    }

    /// Print octet
    void Octet::print() const{
        std::cout << "Camera pose id:" << poseId << "\n";
        std::cout << "Filter ids:\n";
        for (size_t i=0;i<partIds.size();i++){
            for (size_t j=0;j<partIds[i].size();j++){
                std::cout << partIds[i][j] << ", ";
            }
            std::cout << "\n";
        }
        std::cout << "Filter poses (u,v,depth):\n";
        for (size_t i=0;i<filterPos.size();i++){
            for (size_t j=0;j<filterPos[i].size();j++){
                std::cout << "(" << filterPos[i][j].u << "," << filterPos[i][j].v << "," << filterPos[i][j].depth << "), ";
            }
            std::cout << "\n";
        }
    }

    /// compute distance between octets -- dot product for normals for each filter
    double Octet::distance(const Octet& octetA, const Octet& octetB, const Filter::Seq& filters){
        if (filters.size()==0){
            std::cout << "Empty filters set!\n"; getchar();
            return std::numeric_limits<double>::max();
        }
        if (octetA.partIds==octetB.partIds){//fast
            return 0;
        }
        double sum=0;
        for (size_t i=0; i<octetA.partIds.size();i++)
            for (size_t j=0; j<octetA.partIds.size();j++){
                sum+=Filter::distance(filters[octetA.partIds[i][j]], filters[octetB.partIds[i][j]]);
            }
        return sum;
    }

    /// normalize quaternion
    Eigen::Quaterniond& GaussianSE3::normalize(Eigen::Quaterniond& q){
      q.normalize();
      if (q.w()<0) {
        q.coeffs() *= -1;
      }
      return q;
    }

    /// rotation matrix SO(3) to vector3
    Vec3 GaussianSE3::toCompactQuaternion(const Mat33& R) {
      Eigen::Quaterniond q(R);
      normalize(q);
      // return (x,y,z) of the quaternion
      return q.coeffs().head<3>();
    }

    /// rotation matrix from compact quaternion
    Mat33 GaussianSE3::fromCompactQuaternion(const Vec3& v) {
      double w = 1-v.squaredNorm();
      if (w<0)
        return Mat33::Identity();
      else
        w=sqrt(w);
      return Eigen::Quaterniond(w, v[0], v[1], v[2]).toRotationMatrix();
    }

    /// functions to handle the toVector of the whole transformations
    Vec6 GaussianSE3::toVector(const Mat34& t){
        Vec6 out;
        out.block<3,1>(3,0) = toCompactQuaternion(t.rotation());
        out.block<3,1>(0,0) = t.translation();
        return out;
    }

    /// function which creates se3 homogenous transformation from vector
    Mat34 GaussianSE3::fromVector(const Vec6& v){
        Mat34 out;
        Mat33 rot(fromCompactQuaternion(v.block<3,1>(3,0)));
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++)
                out(i,j)=rot(i,j);
        out.translation() = v.block<3,1>(0,0);
        return out;
    }
}
