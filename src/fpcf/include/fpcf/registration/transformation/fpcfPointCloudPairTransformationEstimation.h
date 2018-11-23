/**
 * @file    fpcfPointCloudPairTransformationEstimation.h
 * @author  Thomas Weber
 *
 * Copyright (c) 2014, T. Weber
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _FPCFPOINTCLOUDPAIRTRANSFORMATIONESTIMATION_H_
#define _FPCFPOINTCLOUDPAIRTRANSFORMATIONESTIMATION_H_

// STL

// Boost

// PCL
#include <pcl/registration/transformation_estimation_svd.h>

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{

    /**
     * This class estimates the optimal transformation to reduce the total 
     * distances between the established corresponding points using a singular
     * value decomposition. The correspondences of the point cloud pair need to
     * be computed before using the fpcfCorrespondenceFinder.
     *
     * see paper:
     *   Arun, K.; Huang, T. S. & Blostein, S. D.
     *   "Least-Squares Fitting of Two 3-D Point Sets"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudPairTransformationEstimation
    {
        public:

            /**
             * Constructor that creates a new point cloud pair transformation 
             * estimator for a specified point cloud pair with already computed
             * correspondences.
             *
             * @param[in] pointCloudPair point cloud pair that is used to 
             *                           estimate the transformation
             */
            fpcfPointCloudPairTransformationEstimation(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Constructor that creates a new empts point cloud pair 
             * transformation estimator.
             */
            fpcfPointCloudPairTransformationEstimation();

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudPairTransformationEstimation();

            /**
             * Setter for the point cloud pair, which is used to reduce the 
             * distances between the corresponding points
             *
             * @param[in] pointCloudPair point cloud pair that is used to 
             *                           estimate the transformation
             */
            void setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * This method estimates the transformation, which can be applied 
             * to the source point cloud to reduce the total corresponding 
             * points distance between the point cloud pair. If the point 
             * clouds contain keypoints, then the correspondences between the 
             * keypoints are used to estimate a transformation. Otherwise, the
             * correspondences between all points are used. The computed 
             * transformation is also directly set as transformation between 
             * the point cloud pair and can be accessed by 
             * pointCloudPair->getTransformation().
             *
             * @param[out] transformation the estimated transformation to align
             *                            the source point cloud to the target 
             *                            point cloud of the point cloud pair.
             */
            void estimateAlignment(Eigen::Matrix4f &transformation);

            /**
             * This method estimates the transformation, which can be applied 
             * to the source point cloud to reduce the total distance between 
             * all corresponding points. The computed transformation is also 
             * directly set as transformation between the point cloud pair and
             * can be accessed by pointCloudPair->getTransformation().
             *
             * @param[out] transformation the estimated transformation to align
             *                            the source point cloud to the target 
             *                            point cloud of the point cloud pair.
             */
            void estimateAllPointAlignment(Eigen::Matrix4f &transformation);

            /**
             * This method estimates the transformation, which can be applied 
             * to the source point cloud to reduce the total distance between 
             * the corresponding keypoints. The computed transformation is also
             * directly set as transformation between the point cloud pair and
             * can be accessed by pointCloudPair->getTransformation().
             *
             * @param[out] transformation the estimated transformation to align
             *                            the source point cloud to the target 
             *                            point cloud of the point cloud pair.
             */
            void estimateKeypointAlignment(Eigen::Matrix4f &transformation);

        protected:

        private:
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair_;

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::fpcfPointCloudPairTransformationEstimation()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::fpcfPointCloudPairTransformationEstimation(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        pointCloudPair_ = pointCloudPair;
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::~fpcfPointCloudPairTransformationEstimation()
    {

    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        pointCloudPair_ = pointCloudPair;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::estimateAlignment(Eigen::Matrix4f &transformation)
    {
        if (pointCloudPair_->getPointCloudSource()->getKeypointCloud() && pointCloudPair_->getPointCloudTarget()->getKeypointCloud())
        {
            estimateKeypointAlignment(transformation);
        }
        else
        {
            estimateAllPointAlignment(transformation);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::estimateAllPointAlignment(Eigen::Matrix4f &transformation)
    {
        pcl::registration::TransformationEstimationSVD<PointT, PointT> transEst;
        transEst.estimateRigidTransformation(
            *pointCloudPair_->getPointCloudSource()->getPointCloud(),
            *pointCloudPair_->getPointCloudTarget()->getPointCloud(),
            *pointCloudPair_->getAllPointCorrespondences(),
            transformation
            );
        pointCloudPair_->setTransformation(transformation);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT>::estimateKeypointAlignment(Eigen::Matrix4f &transformation)
    {
        pcl::registration::TransformationEstimationSVD<PointT, PointT> transEst;
        transEst.estimateRigidTransformation(
            *pointCloudPair_->getPointCloudSource()->getKeypointCloud(),
            *pointCloudPair_->getPointCloudTarget()->getKeypointCloud(),
            *pointCloudPair_->getKeypointCorrespondences(),
            transformation
            );
        pointCloudPair_->setTransformation(transformation);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDPAIRTRANSFORMATIONESTIMATION_H_