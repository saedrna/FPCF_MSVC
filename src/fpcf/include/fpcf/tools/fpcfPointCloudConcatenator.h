/**
 * @file    fpcfPointCloudConcatenator.h
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
#ifndef _FPCFPOINTCLOUDCONCATENATOR_H_
#define _FPCFPOINTCLOUDCONCATENATOR_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{

    /**
     * This class concatenates fpcf point cloud objects into a single fpcf point 
     * cloud.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudConcatenator
    {
        public:

            /**
             * Constructor that creates a new empty fpcfPointCloudConcatenator.
             */
            fpcfPointCloudConcatenator();

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudConcatenator();

            /**
             * Adds a point cloud to the concatenation process.
             *
             * @param[in] fpcfPointCloud the new point cloud to add
             */
            void addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);

            /**
             * Sets the vector of point clouds, which will be concatenated into
             * a single point cloud.
             *
             * @param[in] fpcfPointClouds the vector of point clouds, which will
             *                          be concatenated
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /** 
             * Concatenates the input point clouds including the keypoint, 
             * normals and descriptors. The downsampled versions of the point 
             * clouds are not concatenated.
             *
             * @param[out] concatenatedPointCloud the new fpcf point cloud, 
             *                                    containing all points from 
             *                                    the other point clouds
             */
            void concat(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatenatedPointCloud);

        protected:

        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds_;

            template <typename ConcatT>
            typename pcl::PointCloud<ConcatT>::Ptr
            concat(typename pcl::PointCloud<ConcatT>::Ptr fpcfPointCloudSource, typename pcl::PointCloud<ConcatT>::Ptr fpcfPointCloudTarget)
            {
                if (!fpcfPointCloudSource && fpcfPointCloudTarget)
                {
                    fpcfPointCloudSource.reset(new pcl::PointCloud<ConcatT>());
                }
                if (fpcfPointCloudSource && fpcfPointCloudTarget)
                {
                    *fpcfPointCloudSource += *fpcfPointCloudTarget;
                }
                return fpcfPointCloudSource;
            }
    };

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudConcatenator<PointT, DescriptorT>::fpcfPointCloudConcatenator()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT>::~fpcfPointCloudConcatenator()
    {
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT>::addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfPointClouds_.push_back(fpcfPointCloud);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        fpcfPointClouds_ = fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT>::concat(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatenatedPointCloud)
    {
        concatenatedPointCloud->isPrepared(true);
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            concatenatedPointCloud->setPointCloud(concat<PointT>(concatenatedPointCloud->getPointCloud(), fpcfPointClouds_.at(pcNr)->getPointCloud()));
            concatenatedPointCloud->setKeypointCloud(concat<PointT>(concatenatedPointCloud->getKeypointCloud(), fpcfPointClouds_.at(pcNr)->getKeypointCloud()));
            concatenatedPointCloud->setPointDescriptorCloud(concat<DescriptorT>(concatenatedPointCloud->getPointDescriptorCloud(), fpcfPointClouds_.at(pcNr)->getPointDescriptorCloud()));
            concatenatedPointCloud->setKeypointDescriptorCloud(concat<DescriptorT>(concatenatedPointCloud->getKeypointDescriptorCloud(), fpcfPointClouds_.at(pcNr)->getKeypointDescriptorCloud()));
            concatenatedPointCloud->setNormalsCloud(concat<pcl::Normal>(concatenatedPointCloud->getNormalsCloud(), fpcfPointClouds_.at(pcNr)->getNormalsCloud()));
            concatenatedPointCloud->isPrepared(concatenatedPointCloud->isPrepared() && fpcfPointClouds_.at(pcNr)->isPrepared());
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDCONCATENATOR_H_