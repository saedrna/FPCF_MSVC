/**
 * @file    fpcfDetector.h
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

#ifndef _FPCFDETECTOR_H_
#define _FPCFDETECTOR_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * This is an abstract superclass for all point cloud fusion keypoint 
     * detectors.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfDetector
    {
        
        public:
            typedef boost::shared_ptr<fpcfDetector<PointT, DescriptorT>> Ptr;

            /**
             * Destructor
             */ 
            virtual ~fpcfDetector();

            /**
             * Computes the keypoints of a point cloud.
             */
            virtual void detectKeypoints() = 0;

            /**
             * Sets the fpcfPointCloud, which will be used during the keypoint 
             * computation.
             *
             * @param[in] pointCloud the pointCloud, which will be used
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Getter for the current fpcfPointCloud of the keypoint computation
             *
             * @return returns the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

           /**
             * Deep copy
             */
            virtual fpcfDetector<PointT, DescriptorT>* clone() const = 0;

        protected:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;

            fpcfDetector();
            fpcfDetector(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

        private:

    };

    template <typename PointT, typename DescriptorT>
    fpcfDetector<PointT, DescriptorT>::fpcfDetector(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        setPointCloud(pointCloud);
    }

    template <typename PointT, typename DescriptorT>
    fpcfDetector<PointT, DescriptorT>::fpcfDetector()
    {
    }
            
    template <typename PointT, typename DescriptorT>
    fpcfDetector<PointT, DescriptorT>::~fpcfDetector()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfDetector<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfDetector<PointT, DescriptorT>::getPointCloud()
    {
        return pointCloud_;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFDETECTOR_H_