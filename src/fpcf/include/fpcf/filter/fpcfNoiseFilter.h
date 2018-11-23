/**
 * @file    fpcfNoiseFilter.h
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

#ifndef _FPCFNOISEFILTER_H_
#define _FPCFNOISEFILTER_H_

// STL

// Boost

// PCL
#include <pcl/filters/radius_outlier_removal.h>

// FPCF
#include <fpcf/filter/fpcfFilter.h>

namespace fpcf
{

    /**
     * This class filters outlier points in fpcfPointCloudData data by removing 
     * all points of a point cloud, which have less than two point neighbors in
     * a specified radius.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfNoiseFilter : public fpcfFilter<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfNoiseFilter<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfNoiseFilter for a specified 
             * fpcfPointCloud with a specified outliner radius.
             * 
             * @param[in] pointCloud the point cloud, which will be filtered
             * @param[in] outlinerRadius the used radius to search for point
             *                           neighbors
             */
            fpcfNoiseFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, double outlinerRadius);

            /**
             * Constructor that creates a new fpcfNoiseFilter with a specified 
             * outliner radius.
             * 
             * @param[in] outlinerRadius the used radius to search for point 
             *                           neighbors
             */
            fpcfNoiseFilter(double outlinerRadius);

            /**
             * Destructor
             */
            virtual ~fpcfNoiseFilter();

            /**
             * Getter for the current radius of the noise filter. This radius 
             * is used during the point neighbor query.
             *
             * @return returns the current radius of the point neighbors 
             *         collection
             */
            double getOutlinerRadius() const;

            /**
             * Setter for the used radius during the outliner collection. If 
             * point has less than two point neighbors in this radius, it will 
             * be removed.
             *
             * @param[in] size the radius of the point neighbor query
             */
            void setOutlinerRadius(double size);

            /**
             * Deep copy
             */
            fpcfNoiseFilter<PointT, DescriptorT>* clone() const;

            /**
             * Directly removes all points in the point cloud, which have less 
             * than two point neighbors in the specified outliner radius. The 
             * filter is only applied in one pass. Therefore it is possible, 
             * that the point cloud after applying the filter still contains 
             * points with less than two point neighbors.
             */
            void filter();

        protected:

        private:
            double outlinerRadius_;

            fpcfNoiseFilter(); // hide default constructor
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::fpcfNoiseFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, double outlinerRadius)
        : fpcfFilter(pointCloud)
    {
        setOutlinerRadius(outlinerRadius);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::fpcfNoiseFilter(double outlinerRadius)
        : fpcfFilter()
    {
        setOutlinerRadius(outlinerRadius);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::~fpcfNoiseFilter()
    {

    }

    template <typename PointT, typename DescriptorT>
    double
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::getOutlinerRadius() const
    {
        return outlinerRadius_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::setOutlinerRadius(double outlinerRadius)
    {
        outlinerRadius_ = outlinerRadius;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcfNoiseFilter<PointT, DescriptorT>*
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::clone() const
    {
        double outlinerRadius = getOutlinerRadius();
        fpcf::fpcfNoiseFilter<PointT, DescriptorT> *fpcfNoiseFilter = new fpcf::fpcfNoiseFilter<PointT, DescriptorT>(outlinerRadius);
        return fpcfNoiseFilter;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNoiseFilter<PointT, DescriptorT>::filter()
    {
        if (outlinerRadius_ > 0.0001)
        {
            typename pcl::PointCloud<PointT>::Ptr filteredPointCloud(new pcl::PointCloud<PointT>());
            pcl::RadiusOutlierRemoval<PointT> noiseFilter(false);
            noiseFilter.setInputCloud(pointCloud_->getPointCloud());
            noiseFilter.setMinNeighborsInRadius(2);
            noiseFilter.setRadiusSearch(outlinerRadius_);
            noiseFilter.filter(*filteredPointCloud);
            pointCloud_->setPointCloud(filteredPointCloud);
            std::cout << "#NoisePoints removed: " << noiseFilter.getRemovedIndices()->size() << std::endl;
        }
        else
        {
            std::cout << "outliner radius filter too small, skipping noise filtering" << std::endl;
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFNOISEFILTER_H_