/**
 * @file    fpcfFilter.h
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

#ifndef _FPCFFILTER_H_
#define _FPCFFILTER_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * This is an abstract superclass for all point cloud fusion filters.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfFilter
    {
        
        public:
            typedef boost::shared_ptr<fpcfFilter<PointT, DescriptorT>> Ptr;

            /**
             * Destructor
             */
            virtual ~fpcfFilter();

            /**
             * Setter for a fpcfPointCloud, which will be filtered
             *
             * @param[in] pointCloud the fpcfPointCloud
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Getter for the current fpcfPointCloud of this filter
             *
             * @return returns the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

            /**
             * Deep copy
             */
            virtual fpcf::fpcfFilter<PointT, DescriptorT>* clone() const = 0;

            /**
             * This method invokes the filtering of the setted point cloud
             */
            virtual void filter() = 0;

        protected:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;

            /**
             * Constructor that creates a new fpcfFilter. Should be only used by
             * subclasses of fpcfFilter.
             */
            fpcfFilter();

            /**
             * Constructor that creates a new fpcfFilter for a specified point 
             * cloud. Should be only used by subclasses of fpcfFilter.
             *
             * @param pointCloud the pointCloud, which will be filtered
             */
            fpcfFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

        private:
            
    };

    template <typename PointT, typename DescriptorT>
    fpcfFilter<PointT, DescriptorT>::fpcfFilter()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcfFilter<PointT, DescriptorT>::fpcfFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
       setPointCloud(pointCloud);
    }

    template <typename PointT, typename DescriptorT>
    fpcfFilter<PointT, DescriptorT>::~fpcfFilter()
    {
    
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfFilter<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfFilter<PointT, DescriptorT>::getPointCloud()
    {
        return pointCloud_;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFDESCRIPTOR_H_