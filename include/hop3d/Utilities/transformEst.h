/** @file transformEst.h
 *
 * Feature-based transformation estimation interface
 *
 */

#ifndef _TRANSFORMEST_H_
#define _TRANSFORMEST_H_

#include "hop3d/Data/Defs.h"
#include <string>
#include <vector>

namespace putslam {
    /// Transformation Estimator interface
    class TransformEst {
        public:

            /// Name of the estimator
            virtual const std::string& getName() const = 0;

            /// compute transformation using two set of keypoints
            virtual hop3d::Mat34& computeTransformation(const Eigen::MatrixXd& setA, const Eigen::MatrixXd& setB) = 0;

            /// Virtual descrutor
            virtual ~TransformEst() {}

        protected:

            /// Estimated transformation
            hop3d::Mat34 transformation;
            ///Uncertainty
            hop3d::Mat66 uncertainty;
    };
};

#endif // _TRANSFORMEST_H_

