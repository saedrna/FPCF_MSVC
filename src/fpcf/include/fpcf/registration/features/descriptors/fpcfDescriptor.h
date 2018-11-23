/**
 * @file    fpcfDescriptor.h
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

#ifndef _FPCFDESCRIPTOR_H_
#define _FPCFDESCRIPTOR_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * This is an abstract superclass for all point cloud fusion point 
     * descriptors.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfDescriptor
    {
        
        public:
            typedef boost::shared_ptr<fpcfDescriptor<PointT, DescriptorT>> Ptr;

            /**
             * Destructor
             */
            virtual ~fpcfDescriptor();

            /**
             * Computes the point descriptors of all points of the point cloud.
             */
            virtual void calculatePointDescriptors() = 0;

            /**
             * Computes the point descriptors for the keypoints of the point 
             * cloud.
             */
            virtual void calculateKeypointDescriptors() = 0;

            /**
             * Sets the fpcfPointCloud, which will be used during the point 
             * descriptor computation.
             *
             * @param[in] pointCloud the pointCloud, which will be used
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Getter for the current fpcfPointCloud of the point descriptor 
             * computation
             *
             * @return returns the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

            /**
             * Sets the radius, which will be used to describe the points of 
             * the point cloud.
             *
             * @param[in] searchRadius the search radius of the descriptor 
             *                         computation
             */
            virtual void setSearchRadius(double searchRadius) = 0;

            /**
             * Gets the current search radius of the descriptor computation
             *
             * @return returns the current search radius
             */
            virtual double getSearchReadius() const = 0;

            /**
             * Deep copy
             */
            virtual fpcfDescriptor<PointT, DescriptorT>* clone() const = 0;

        protected:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;

            fpcfDescriptor();
            fpcfDescriptor(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

        private:
            
    };

    template <typename PointT, typename DescriptorT>
    fpcfDescriptor<PointT, DescriptorT>::fpcfDescriptor(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        setPointCloud(pointCloud);
    }

    template <typename PointT, typename DescriptorT>
    fpcfDescriptor<PointT, DescriptorT>::fpcfDescriptor()
    {
    }
            
    template <typename PointT, typename DescriptorT>
    fpcfDescriptor<PointT, DescriptorT>::~fpcfDescriptor()
    {
    
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfDescriptor<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfDescriptor<PointT, DescriptorT>::getPointCloud()
    {
        return pointCloud_;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFDESCRIPTOR_H_