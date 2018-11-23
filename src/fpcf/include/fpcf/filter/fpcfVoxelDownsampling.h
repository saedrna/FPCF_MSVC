/**
 * @file    fpcfVoxelDownsampling.h
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

#ifndef _FPCFVOXELDOWNSAMPLING_H_
#define _FPCFVOXELDOWNSAMPLING_H_

// STL

// Boost

// PCL
#include <pcl/filters/voxel_grid.h>

// FPCF
#include <fpcf/filter/fpcfFilter.h>

namespace fpcf
{

    /**
     * This class downsamples fpcfPointCloudData. The new point cloud is set as 
     * downsampledPointCloud of the input point cloud and can be accessed via
     * fpcfPointCloudData->getDownsampledPointCloud()
     */
    template <typename PointT, typename DescriptorT>
    class fpcfVoxelDownsampling : public fpcfFilter<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfVoxelDownsampling<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a voxelgrid filter with a specified fpcfPointCloud and leafSize
             *
             * @param[in] fpcfPointCloud sets the point cloud, which will be 
             *                          downsampled
             * @param[in] leafSize sets the used grid size for downsampling
             */
            fpcfVoxelDownsampling(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, float leafSize);

            /**
             * Constructor that creates a voxelgrid filter with a specified 
             * leafSize
             *
             * @param[in] leafSize sets the used grid size for downsampling
             */
            fpcfVoxelDownsampling(float leafSize);

            /**
             * Destructor
             */
            virtual ~fpcfVoxelDownsampling();

            /**
             * Getter for the current leaf size
             *
             * @return returns the current leaf size
             */
            float getLeafSize() const;

            /**
             * Setter for the leaf size
             *
             * @param[in] size the new leaf size
             */
            void setLeafSize(float size);

            /**
             * Deep copy
             */
            fpcfVoxelDownsampling<PointT, DescriptorT>* clone() const;

            /**
             * Creates a new downsampled version of the input fpcfPointCloud. 
             * The downsampled point cloud is set as the new downsampled 
             * point cloud of the input point cloud and can be accessed by
             * fpcfPointCloudData->getDownsampledPointCloud()
             */
            void filter();

        protected:

        private:
            float leafSize_;
            fpcfVoxelDownsampling(); // hide default constructor
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::fpcfVoxelDownsampling(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, float leafSize)
        : fpcfFilter(fpcfPointCloud)
    {
        setLeafSize(leafSize);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::fpcfVoxelDownsampling(float leafSize)
        : fpcfFilter()
    {
        setLeafSize(leafSize);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::~fpcfVoxelDownsampling()
    {

    }

    template <typename PointT, typename DescriptorT>
    float
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::getLeafSize() const
    {
        return leafSize_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::setLeafSize(float leafSize)
    {
        leafSize_ = leafSize;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcfVoxelDownsampling<PointT, DescriptorT>*
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::clone() const
    {
        float leafSize = getLeafSize();
        fpcf::fpcfVoxelDownsampling<PointT, DescriptorT> *fpcfDownsampler = new fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>(leafSize);
        return fpcfDownsampler;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::filter()
    {
        if (leafSize_ > 0.0001)
        {
            pcl::VoxelGrid<PointT> filter;
            pcl::PointCloud<PointT>::Ptr downsampledPointCloud(new pcl::PointCloud<PointT>());
            filter.setInputCloud(pointCloud_->getPointCloud());
            filter.setLeafSize(leafSize_, leafSize_, leafSize_);
            filter.filter(*downsampledPointCloud);
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr downsampledPcfPointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(downsampledPointCloud, pointCloud_->getId() + "downsampled"));
            pointCloud_->setDownsampledPointCloud(downsampledPcfPointCloud);
            std::cout << "PointCloud after downsampling: " << downsampledPointCloud->width * downsampledPointCloud->height << std::endl;
        }
        else
        {
            std::cout << "Voxelgridsize too small, skipping voxelgrid downsampling " << std::endl;
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFVOXELDOWNSAMPLING_H_