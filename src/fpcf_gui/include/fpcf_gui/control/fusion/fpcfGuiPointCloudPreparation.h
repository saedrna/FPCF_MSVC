/**
 * @file    fpcfGuiPointCloudPreparation.h
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

#ifndef _FPCFGUIPOINTCLOUDPREPARATION_H_
#define _FPCFGUIPOINTCLOUDPREPARATION_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/registration/fpcfPointCloudPreparation.h>
#include <fpcf/filter/fpcfDistanceFilter.h>
#include <fpcf/filter/fpcfNoiseFilter.h>
#include <fpcf/filter/fpcfPlaneFilter.h>
#include <fpcf/filter/fpcfVoxelDownsampling.h>
#include <fpcf/registration/features/descriptors/fpcfFPFHDescriptor.h>

namespace fpcf_gui
{
    /**
     * This class is a specialization of the fpcfPointCloudPreparation class for 
     * the fpcfGUI. It already adds point cloud filters and a default point 
     * descriptor.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfGuiPointCloudPreparation : public fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfGuiPointCloudPreparation<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new empty fpcfGuiPointCloudPreparation
             * object.
             */
            fpcfGuiPointCloudPreparation();

            /**
             * Destructor
             */
            virtual ~fpcfGuiPointCloudPreparation();

            /**
             * Getter for the current size of the voxelgrid
             *
             * @return returns the current voxelgrid size
             */
            double getVoxelgridSize();

            /**
             * Getter for the current filter distance
             *
             * @return returns the current filter distance
             */
            float getFilterDistance();

            /**
             * Getter for the maximum point number of a plane
             *
             * @return returns the current maximum number of points a plane can
             *                 consist.
             */
            int getMaxPlaneSize();

            /**
             * Getter for the current radius of the noise filter. This radius 
             * is used during the point neighbor query.
             *
             * @return returns the current radius of the point neighbors 
             *         collection
             */
            double getOutlinerRadius();
            
            /**
             * Gets the current search radius of the descriptor computation
             *
             * @return returns the current search radius
             */
            double getDescriptorRadius();

            /**
             * Setter for the used size of the voxelgrid filter
             *
             * @param[in] leafSize the new size of the voxelgrid
             */
            void setVoxelgridSize(double leafSize);

            /**
             * Setter for filtering distance of the distance filter
             *
             * @param[in] filterDistance the new filter distance
             */
            void setFilterDistance(float filterDistance);

            /**
             * Setter for the maximum number of points a plane can consist
             *
             * @param[in] maxPlaneSize the maximum number of points a plane is 
             *                         allowed to contain
             */
            void setMaxPlaneSize(int maxPlaneSize);

            /**
             * Setter for the used radius during the outliner collection. If 
             * point has less than two point neighbors in this radius, it will 
             * be removed.
             *
             * @param[in] outlinerRadius the radius of the point neighbor query
             */
            void setOutlinerRadius(double outlinerRadius);

            /**
             * Sets the radius, which will be used to describe the points of 
             * the point cloud.
             *
             * @param[in] descriptorRadius the search radius of the descriptor 
             *                             computation
             */
            void setDescriptorRadius(double descriptorRadius);

        protected:

        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds_;
            typename fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>::Ptr fpcfDownsampler_;
            typename fpcf::fpcfDistanceFilter<PointT, DescriptorT>::Ptr fpcfDistanceFilter_;
            typename fpcf::fpcfPlaneFilter<PointT, DescriptorT>::Ptr fpcfPlaneFilter_;
            typename fpcf::fpcfNoiseFilter<PointT, DescriptorT>::Ptr fpcfNoiseFilter_;

            void init();

    };

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::fpcfGuiPointCloudPreparation()
        : fpcfPointCloudPreparation()
    {
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::~fpcfGuiPointCloudPreparation()
    {
    }

    template <typename PointT, typename DescriptorT>
    double
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::getVoxelgridSize()
    {
        return fpcfDownsampler_->getLeafSize();
    }

    template <typename PointT, typename DescriptorT>
    float
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::getFilterDistance()
    {
        return fpcfDistanceFilter_->getFilterDistance();
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::getMaxPlaneSize()
    {
        return fpcfPlaneFilter_->getMaxPlaneSize();
    }

    template <typename PointT, typename DescriptorT>
    double
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::getOutlinerRadius()
    {
        return fpcfNoiseFilter_->getSearchRadius();
    }

    template <typename PointT, typename DescriptorT>
    double
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::getDescriptorRadius()
    {
        fpcfFPFHDescriptor_->getDescriptorRadius();
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::setVoxelgridSize(double leafSize)
    {
        fpcfDownsampler_->setLeafSize(leafSize);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::setFilterDistance(float filterDistance)
    {
        fpcfDistanceFilter_->setFilterDistance(filterDistance);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::setMaxPlaneSize(int maxPlaneSize)
    {
        fpcfPlaneFilter_->setMaxPlaneSize(maxPlaneSize);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::setOutlinerRadius(double outlinerRadius)
    {
        fpcfNoiseFilter_->setOutlinerRadius(outlinerRadius);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::setDescriptorRadius(double descriptorRadius)
    {
        getDescriptor()->setSearchRadius(descriptorRadius);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::init()
    {
        const float DEFAULT_VOXEL_LEAF_SIZE = 0.01f;
        const float DEFAULT_FILTER_DISTANCE = 1.5;
        const int DEFAULT_MAX_PLANE_SIZE = 1000;
        const float DEFAULT_NOISE_OUTLINER_RADIUS = 0.02f;

        // use persistent features instead of explicit keypoint detection
        calculatePersistentFeature(true);

        // add filters
        fpcfDownsampler_.reset(new fpcf::fpcfVoxelDownsampling<PointT, DescriptorT>(DEFAULT_VOXEL_LEAF_SIZE));
        fpcfDistanceFilter_.reset(new fpcf::fpcfDistanceFilter<PointT, DescriptorT>(fpcf::Direction::Z, DEFAULT_FILTER_DISTANCE));
        fpcfPlaneFilter_.reset(new fpcf::fpcfPlaneFilter<PointT, DescriptorT>(DEFAULT_MAX_PLANE_SIZE));
        fpcfNoiseFilter_.reset(new fpcf::fpcfNoiseFilter<PointT, DescriptorT>(DEFAULT_NOISE_OUTLINER_RADIUS));
        addFilter(fpcfDistanceFilter_);
        addFilter(fpcfDownsampler_);
        addFilter(fpcfPlaneFilter_);
        addFilter(fpcfNoiseFilter_);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFGUIPOINTCLOUDPREPARATION_H_