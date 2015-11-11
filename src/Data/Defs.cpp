#include "Data/Defs.h"

namespace hop3d {

    /// compute distance between filters -- dot product for normals
    double Filter::distance (const Filter& filterA, const Filter& filterB){
        return (filterA.id==filterB.id) ? 0 : (double)(1.0-filterA.normal.adjoint()*filterB.normal)/2.0;
    }

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const ImageCoordsDepth& coords){
        //save coords
        os << coords.depth << " " << coords.u << " " << coords.v << "\n";
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, ImageCoordsDepth& coords){
        is >> coords.depth >> coords.u >> coords.v;
        return is;
    }

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const Filter& filter){
        os << filter.id;
        os << " " << filter.normal(0) << " " << filter.normal(1) << " " << filter.normal(2) << " ";
        os << filter.mask.rows << " " << filter.mask.cols << " ";
        for (int i=0;i<filter.mask.rows;i++){
            for (int j=0;j<filter.mask.cols;j++){
                os << filter.mask.at<double>(i,j) << " ";
            }
        }
        os << filter.patch.rows << " " << filter.patch.cols << " ";
        for (int i=0;i<filter.patch.rows;i++){
            for (int j=0;j<filter.patch.cols;j++){
                os << filter.patch.at<double>(i,j) << " ";
            }
        }
        os << "\n";
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, Filter& filter){
        is >> filter.id;
        is >> filter.normal(0) >> filter.normal(1) >> filter.normal(2);
        int cols, rows;
        is >> rows >> cols;
        cv::Mat maskTmp(cols,rows, cv::DataType<double>::type,1);
        for (int i=0;i<rows;i++){
            for (int j=0;j<cols;j++){
                is >> maskTmp.at<double>(i,j);
            }
        }
        filter.mask = maskTmp.clone();
        is >> rows >> cols;
        cv::Mat patchTmp(cols,rows, cv::DataType<double>::type,1);
        for (int i=0;i<rows;i++){
            for (int j=0;j<cols;j++){
                is >> patchTmp.at<double>(i,j);
            }
        }
        filter.patch = patchTmp.clone();
        return is;
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

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const Gaussian3D& gaussian){
        //save gaussian 3d
        os << gaussian.mean(0) << " " << gaussian.mean(1) << " " << gaussian.mean(2) << "\n";
        for (int i=0;i<gaussian.covariance.rows();i++){
            for (int j=0;j<gaussian.covariance.cols();j++){
                os << gaussian.covariance(i,j) << " ";
            }
        }
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, Gaussian3D& gaussian){
        // read gaussian 3d
        is >> gaussian.mean(0) >> gaussian.mean(1) >> gaussian.mean(2);
        for (int i=0;i<gaussian.covariance.rows();i++){
            for (int j=0;j<gaussian.covariance.cols();j++){
                is >> gaussian.covariance(i,j);
            }
        }
        return is;
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

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const GaussianSE3& gaussian){
        //save filters no
        /*os << hierarchy.firstLayer.size() << "\n";
        for (auto& filter : hierarchy.firstLayer){
            os << filter;
        }
        os << hierarchy.viewDependentLayers.size() << "\n";
        for (size_t i = 0;i<hierarchy.viewDependentLayers.size();i++){
            os << hierarchy.viewDependentLayers[i].size() << "\n";
            for (auto& part : hierarchy.viewDependentLayers[i]){
                os << part;
            }
        }*/
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, GaussianSE3& gaussian){
        // read filters no
        /*int filtersNo;
        is >> filtersNo;
        hierarchy.firstLayer.clear();
        hierarchy.firstLayer.reserve(filtersNo);
        for (int i=0;i<filtersNo;i++){
            Filter filterTmp;
            is >> filterTmp;
            hierarchy.firstLayer.push_back(filterTmp);
        }
        int viewDepLayersNo;
        is >> viewDepLayersNo;
        hierarchy.viewDependentLayers.clear();
        hierarchy.viewDependentLayers.resize(viewDepLayersNo);
        for (int i = 0;i<viewDepLayersNo;i++){
            int partsNo;
            is >> partsNo;
            for (int j = 0;j<partsNo;j++){
                ViewDependentPart part;
                is >> part;
                hierarchy.viewDependentLayers[i].push_back(part);
            }
        }*/
        return is;
    }
}
