/**
 * @file    fpcfPlaneFilter.h
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

#ifndef _FPCFPLANEFILTER_H_
#define _FPCFPLANEFILTER_H_

// STL

// Boost

// PCL
#include <pcl/filters/extract_indices.h>

// FPCF
#include <fpcf/filter/fpcfFilter.h>
#include <fpcf/segmentation/fpcfPlaneSegmentation.h>

namespace fpcf
{
    /**
     * This class removes all planes from fpcfPointCloudData, which consists 
     * more points than a specified threshold.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPlaneFilter : public fpcfFilter<PointT, DescriptorT>
    {
        
        public:
            typedef boost::shared_ptr<fpcfPlaneFilter<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new plane filter for a specified 
             * point cloud with a specified maximum plane size.
             *
             * @param[in] pointCloud the used fpcfPointCloud during the plane 
             *                   filtering
             * @param[in] maxPlaneSize the maximum number of points a plane is 
             *                     allowed to contain
             */
            fpcfPlaneFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, int maxPlaneSize);

            /**
             * Constructor that creates a new plane filter with a specified 
             * maximum plane size.
             *
             * @param[in] maxPlaneSize the maximum number of points a plane is 
             *                     allowed to contain
             */
            fpcfPlaneFilter(int maxPlaneSize);

            /**
             * Destructor
             */
            virtual ~fpcfPlaneFilter();

            /**
             * Setter for the maximum number of points a plane can consist
             *
             * @param[in] maxPlaneSize the maximum number of points a plane is 
             *                         allowed to contain
             */
            void setMaxPlaneSize(int maxPlaneSize);

            /**
             * Getter for the maximum point number of a plane
             *
             * @return returns the current maximum number of points a plane can
             *                 consist.
             */
            int getMaxPlaneSize() const;

            /**
             * Deep copy
             */
            fpcf::fpcfPlaneFilter<PointT, DescriptorT>* clone() const;

            /**
             * This starts the actual filtering process of the point cloud. 
             * After applying this filter, all planes of the point cloud are 
             * removed, which contain more points than the specified number of
             * maximum plane size.
             */
            void filter();

        protected:
            
        private:
            int maxPlaneSize_;

            fpcfPlaneFilter(); // hide default constructor

    };

    template <typename PointT, typename DescriptorT>
    fpcfPlaneFilter<PointT, DescriptorT>::fpcfPlaneFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, int maxPlaneSize)
        : fpcfFilter(pointCloud)
    {
        setMaxPlaneSize(maxPlaneSize);
    }

    template <typename PointT, typename DescriptorT>
    fpcfPlaneFilter<PointT, DescriptorT>::fpcfPlaneFilter(int maxPlaneSize)
        : fpcfFilter()
    {
        setMaxPlaneSize(maxPlaneSize);
    }
            
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>::~fpcfPlaneFilter()
    {
    
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>::setMaxPlaneSize(int maxPlaneSize)
    {
        maxPlaneSize_ = maxPlaneSize;
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>::getMaxPlaneSize() const
    {
        return maxPlaneSize_;
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>*
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>::clone() const
    {
        int maxPlaneSize = getMaxPlaneSize();
        fpcf::fpcfPlaneFilter<PointT, DescriptorT> *fpcfPlaneFilter = new fpcf::fpcfPlaneFilter<PointT, DescriptorT>(maxPlaneSize);
        return fpcfPlaneFilter;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPlaneFilter<PointT, DescriptorT>::filter()
    {
        int nrPlaneIndices;
        if (maxPlaneSize_ > 0)
        {
            do {
                fpcf::fpcfPlaneSegmentation<PointT, DescriptorT> fpcfPlaneSeg(pointCloud_);
                pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
                pcl::ExtractIndices<PointT> indicesEx(false);
                indicesEx.setNegative(true);
                fpcfPlaneSeg.findPlane(planeInliers);
                nrPlaneIndices = planeInliers.get()->indices.size();
                if (nrPlaneIndices >= maxPlaneSize_)
                {
                    pcl::PointCloud<PointT>::Ptr newPointCloud(new pcl::PointCloud<PointT>());
                    indicesEx.setInputCloud(pointCloud_->getPointCloud());
                    indicesEx.setIndices(planeInliers);
                    indicesEx.filter(*newPointCloud);
                    pointCloud_->setPointCloud(newPointCloud);
                    std::cout << "removed #PlanePoints: " << nrPlaneIndices << std::endl;
                }
            } while (nrPlaneIndices >= maxPlaneSize_);
        }
        else
        {
            std::cout << "max plane size too small, skipping plane filtering" << std::endl;
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPLANEFILTER_H_