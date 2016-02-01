#include "hop3d/Data/Defs.h"
#include <cmath>
#include <float.h>

namespace hop3d {

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const Mat34& mat){
        for (int i=0;i<3;i++){
            for (int j=0;j<4;j++){
                #ifdef _WIN32
				if (_isnan(double(mat(i, j))))
                #else
                if (std::isnan(mat(i,j)))
                #endif
                    os << 0.0 << " ";
                else
                    os << mat(i,j) << " ";
            }
        }
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, Mat34& mat){
        for (int i=0;i<3;i++){
            for (int j=0;j<4;j++){
                is >> mat(i,j);
            }
        }
        return is;
    }

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
        //std::cout << "Camera pose id:" << poseId << "\n";
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
        for (size_t i=0;i<this->partsPosEucl.size();i++){
            for (size_t j=0;j<partsPosEucl[i].size();j++){
                std::cout << "(" << partsPosEucl[i][j](0) << "," << partsPosEucl[i][j](1) << "," << partsPosEucl[i][j](2) << "), ";
            }
            std::cout << "\n";
        }
    }

    /// check if the octet contains double surface
    bool Octet::hasDoubleSurface(double distThreshold, int& groupSize, int& othersSize) const{
        std::vector<double> depth;
        for (int i=0;i<3;i++){//detect two surfaces
            for (int j=0;j<3;j++){
                if (partIds[i][j]>=0)
                    depth.push_back(partsPosNorm[i][j].mean(2));
            }
        }
        std::sort(depth.begin(),depth.end());
        for (size_t i=0;i<depth.size()-1;i++){
            if (depth[i+1]-depth[i]>distThreshold){
                if (i+1>depth.size()-(i+1))
                    groupSize = int(i+1);
                else
                    groupSize = int(depth.size()-(i+1));
                othersSize = (int)depth.size()-groupSize;
                return true;
            }
        }
        groupSize = (int)depth.size();
        othersSize = 0;
        return false;
    }

/// split double surfaces
void Octet::splitSurfaces(double distThreshold, int minOctetSize, int smallerGroupSize){
        std::vector<double> depth;
        for (int i=0;i<3;i++){//detect two surfaces
            for (int j=0;j<3;j++){
                if (partIds[i][j]>=0){
                    if (i==1&&j==1)
                        depth.push_back(0);
                    else
                        depth.push_back(partsPosNorm[i][j].mean(2));
                }
            }
        }
        std::sort(depth.begin(),depth.end());
        double distThr=0;
        int isBackground(true);
        for (size_t i=0;i<depth.size()-1;i++){
            if (depth[i+1]-depth[i]>distThreshold){
                if (i+1>depth.size()-(i+1))
                    isBackground = true;
                else
                    isBackground = false;
                distThr = depth[i];
                break;
            }
        }
        // create second octet
        if (smallerGroupSize>=minOctetSize){
            Octet background;
            Vec3 meanPos(0,0,0);
            for (int i=0;i<3;i++){//detect two surfaces
                for (int j=0;j<3;j++){
                    double dist;
                    if (i==1&&j==1)
                        dist = 0;
                    else
                        dist = partsPosNorm[i][j].mean(2);
                    if (partIds[i][j]>=0&&dist>distThr&&isBackground){
                        background.partsPosNorm[i][j] = partsPosNorm[i][j];
                        background.partsPosEucl[i][j] = partsPosEucl[i][j];
                        background.partIds[i][j] = partIds[i][j];
                        partIds[i][j] = -2;
                        background.offsets[i][j] = offsets[i][j];
                        background.filterPos[i][j] = filterPos[i][j];
                        meanPos+=background.partsPosNorm[i][j].mean.block<3,1>(0,0);
                    }
                    if (partIds[i][j]>=0&&dist<(distThr+std::numeric_limits<double>::epsilon())&&!isBackground){
                        background.partsPosNorm[i][j] = partsPosNorm[i][j];
                        background.partsPosEucl[i][j] = partsPosEucl[i][j];
                        background.partIds[i][j] = partIds[i][j];
                        partIds[i][j] = -2;
                        background.offsets[i][j] = offsets[i][j];
                        background.filterPos[i][j] = filterPos[i][j];
                        meanPos+=background.partsPosNorm[i][j].mean.block<3,1>(0,0);
                    }
                }
            }
            Vec3 middleElement(0,0,0);
            if (background.partIds[1][1]<0){
                middleElement = meanPos*double(1.0/double(smallerGroupSize));
            }
            else{
                middleElement = background.partsPosNorm[1][1].mean.block<3,1>(0,0);
            }
            background.partsPosNorm[1][1].mean.block<3,1>(0,0) =partsPosNorm[1][1].mean.block<3,1>(0,0)+middleElement;
            background.partsPosEucl[1][1] =partsPosEucl[1][1]+middleElement;
            for (int i=0;i<3;i++)
                for (int j=0;j<3;j++)
                    if ((background.partIds[i][j]>=0)&&(!(i==1&&j==1)))
                        background.partsPosNorm[i][j].mean.block<3,1>(0,0)-=middleElement;
            background.isBackground = false;
            secondOctet.push_back(background);
        }
        else{// clean second surface
            for (int i=0;i<3;i++){
                for (int j=0;j<3;j++){
                    double dist;
                    if (i==1&&j==1)
                        dist = 0;
                    else
                        dist = partsPosNorm[i][j].mean(2);
                    if (partIds[i][j]>=0&&dist>distThr&&isBackground){
                        partIds[i][j] = -2;
                    }
                    if (partIds[i][j]>=0&&dist<=(distThr+std::numeric_limits<double>::epsilon())&&!isBackground){
                        partIds[i][j] = -2;
                    }
                }
            }
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
    std::ostream& operator<<(std::ostream& os, const Octet& octet){
        //save octet
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                os << octet.partIds[i][j] << " ";
            }
        }
        os << "\n";
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                os << octet.realisationsIds[i][j] << " ";
            }
        }
        os << "\n";
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                os << octet.partsPosNorm[i][j] << " ";
            }
        }
        os << "\n";
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                os << octet.partsPosEucl[i][j] << " ";
            }
        }
        os << "\n";
        os << octet.isBackground << " ";
        if (octet.secondOctet.size()>0){
            os << 1 << " ";
            os << octet.secondOctet[0];
        }
        else
            os << 0;
        os << "\n";
        return os;
    }

    // Extraction operator
    std::istream& operator>>(std::istream& is, Octet& octet){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                is >> octet.partIds[i][j];
            }
        }
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                is >> octet.realisationsIds[i][j];
            }
        }
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                is >> octet.partsPosNorm[i][j];
            }
        }
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                is >> octet.partsPosEucl[i][j];
            }
        }
        is >> octet.isBackground;
        int isSecond;
        is >> isSecond;
        if (isSecond){
            Octet secondOct;
            is >> secondOct;
            octet.secondOctet.push_back(secondOct);
        }
        return is;
    }

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const Gaussian3D& gaussian){
        //save gaussian 3d
        for (int i=0;i<3;i++){
			if (std::isnan(double(gaussian.mean(i))))
                os << 0.0 << " ";
            else
                os << gaussian.mean(i) << " ";
        }
        for (int i=0;i<gaussian.covariance.rows();i++){
            for (int j=0;j<gaussian.covariance.cols();j++){
				if (std::isnan(double(gaussian.covariance(i, j))))
                    os << 0.0 << " ";
                else
                    os << gaussian.covariance(i,j) << " ";
            }
        }
        os << "\n";
        return os;
    }

    // Insertion operator
    std::ostream& operator<<(std::ostream& os, const Vec3& vec){
        for (int i=0;i<3;i++){
            if (std::isnan(double(vec(i))))
                os << 0.0 << " ";
            else
                os << vec(i) << " ";
        }
        os << "\n";
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

    // Extraction operator
    std::istream& operator>>(std::istream& is, Vec3& vec){
        is >> vec(0) >> vec(1) >> vec(2);
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

    /// Insertion operator
    std::ostream& operator<<(std::ostream& os, const GaussianSE3& gaussian){
        //mean value
        for (int i=0;i<gaussian.mean.rows();i++){
			if (std::isnan(double(gaussian.mean(i))))
                os << 0.0 << " ";
            else
                os << gaussian.mean(i) << " ";
        }
        // covariance
        for (int i=0;i<gaussian.covariance.rows();i++){
            for (int j=0;j<gaussian.covariance.rows();j++){
				if (std::isnan(double(gaussian.covariance(i, j))))
                    os << 0.0 << " ";
                else
                    os << gaussian.covariance(i,j) << " ";
            }
        }
        os << "\n";
        return os;
    }

    /// Extraction operator
    std::istream& operator>>(std::istream& is, GaussianSE3& gaussian){
        // read mean
        for (int i=0;i<gaussian.mean.rows();i++)
            is >> gaussian.mean(i);
        // covariance
        for (int i=0;i<gaussian.covariance.rows();i++){
            for (int j=0;j<gaussian.covariance.rows();j++){
                is >> gaussian.covariance(i,j);
            }
        }
        return is;
    }
}
