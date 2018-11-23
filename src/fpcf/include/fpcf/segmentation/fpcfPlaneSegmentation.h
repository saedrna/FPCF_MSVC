/**
 * @file    fpcfPlaneSegmentation.h
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

#ifndef _FPCFPLANESEGMENTATION_H_
#define _FPCFPLANESEGMENTATION_H_

// STL

// Boost

// PCL
#include <pcl/segmentation/sac_segmentation.h>

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{

    /**
     * This class finds the biggest plane in fpcfPointCloudData using a RANSAC 
     * method.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPlaneSegmentation
    {
        public:

            /**
             * Constructor that creates a new plane segmentator to find the 
             * biggest plane in a specified point cloud.
             *
             * @param[in] pointCloud the input point cloud for the plane 
             *                       segmentation
             */
            fpcfPlaneSegmentation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Destructor
             */
            virtual ~fpcfPlaneSegmentation();

            /**
             * Invokes the computation of the biggest plane in the point cloud 
             * using a RANSAC method. The output of this method are the indices
             * of points, which are lying on the biggest plane of the point 
             * cloud.
             *
             * @param[out] planeInliers the point indices on the biggest plane 
             *                          found
             */
            void findPlane(pcl::PointIndices::Ptr planeInliers);

        protected:

        private:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;

            fpcfPlaneSegmentation(); // hide default constructor

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPlaneSegmentation<PointT, DescriptorT>::fpcfPlaneSegmentation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPlaneSegmentation<PointT, DescriptorT>::~fpcfPlaneSegmentation()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPlaneSegmentation<PointT, DescriptorT>::findPlane(pcl::PointIndices::Ptr planeInliers)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<PointT> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setMaxIterations(50);
        seg.setInputCloud(pointCloud_->getPointCloud());
        seg.segment(*planeInliers, *coefficients);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPLANESEGMENTATION_H_