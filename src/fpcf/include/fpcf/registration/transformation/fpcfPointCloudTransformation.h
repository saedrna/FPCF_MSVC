/**
 * @file    fpcfPointCloudTransformation.h
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

#ifndef _FPCFPOINTCLOUDTRANSFORMATION_H_
#define _FPCFPOINTCLOUDTRANSFORMATION_H_

// STL

// Boost

// PCL
#include <pcl/common/transforms.h>

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{

    /**
     * This class applies transformations to fpcfPointCloudData data.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudTransformation
    {
        public:

            /**
             * Constructor that creates a new fpcfPointCloudTransformation for a
             * specified point cloud.
             * 
             * @param[in] pointCloud the point cloud which will be transformed
             */
            fpcfPointCloudTransformation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Constructor that creates a new emmpty 
             * fpcfPointCloudTransformation.
             */
            fpcfPointCloudTransformation();

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudTransformation();

            /**
             * Getter for the transformation, which will be applied to the 
             * point cloud.
             *
             * @return returns the current transformation
             */
            Eigen::Matrix4f getTransformation();

            /**
             * Setter for the fpcfPointCloud, which will be transformed.
             *
             * @param[in] pointCloud the point cloud for the transformation
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Setter for the used transformation
             *
             * @param[in] transformation the transformation, which will be 
             *                           applied to the point cloud
             */
            void setTransformation(Eigen::Matrix4f &transformation);

            /**
             * This method directly transforms the point cloud. If the point 
             * cloud contains keypoints, they will be also transformed. This 
             * method doesn't transform downsampled versions of the point 
             * cloud.
             */
            void transformPointCloud();

            /**
             * This method directly transforms the downsampled version of the 
             * point cloud. If the downsampled version point cloud contains 
             * keypoints, they will be also transformed.
             */
            void transformDownsampledPointCloud();

            /**
             * This method directly transforms the most downsampled version of 
             * the point cloud. If the most downsampled point cloud contains 
             * keypoints, they will be also transformed.
             */
            void transformMostDownsampledPointCloud();

        protected:

        private:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;
            Eigen::Matrix4f transformation_;

            void transformPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::fpcfPointCloudTransformation()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::fpcfPointCloudTransformation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::~fpcfPointCloudTransformation()
    {

    }

    template <typename PointT, typename DescriptorT>
    Eigen::Matrix4f fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::getTransformation()
    {
        return transformation_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::setTransformation(Eigen::Matrix4f &transformation)
    {
        transformation_ = transformation;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::transformPointCloud()
    {
        transformPointCloud(pointCloud_);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::transformDownsampledPointCloud()
    {
        if (pointCloud_->getDownsampledPointCloud())
        {
            transformPointCloud(pointCloud_->getDownsampledPointCloud());
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::transformMostDownsampledPointCloud()
    {
        transformPointCloud(pointCloud_->getMostDownsampledPointCloud());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudTransformation<PointT, DescriptorT>::transformPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pcl::transformPointCloud(*(pointCloud->getPointCloud()), *(pointCloud->getPointCloud()), transformation_);
        if (pointCloud->getKeypointCloud())
        {
            pcl::transformPointCloud(*(pointCloud->getKeypointCloud()), *(pointCloud->getKeypointCloud()), transformation_);
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDTRANSFORMATION_H_