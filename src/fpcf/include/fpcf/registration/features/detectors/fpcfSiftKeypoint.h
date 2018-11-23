/**
 * @file    fpcfSiftKeypoint.h
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

#ifndef _FPCFSIFTKEYPOINTS_H_
#define _FPCFSIFTKEYPOINTS_H_

// STL
#include <exception>

// Boost

// PCL
#include <pcl/keypoints/sift_keypoint.h>

// FPCF
#include <fpcf/registration/features/detectors/fpcfDetector.h>

namespace fpcf
{

    /**
     * This class calculates SIFT keypoints for fpcfPointCloudData.  The 
     * normals of the fpcfPointCloud need to computed beforehand for the SIFT
     * keypoint computation.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfSiftKeypoint : public fpcfDetector<PointT, DescriptorT>
    {
        public:

            /**
             * Constructor that creates a new fpcfSiftKeypoint detector for a 
             * specified point cloud.
             *
             * @param[in] pointCloud the used point cloud during the 
             *                       comptutation
             */
            fpcfSiftKeypoint(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Constructor that creates a new empty fpcfNarfKeypoint detector
             */
            fpcfSiftKeypoint();

            /**
             * Destructor
             */
            virtual ~fpcfSiftKeypoint();

            /**
             * Deep copy
             */
            fpcfSiftKeypoint<PointT, DescriptorT>* clone() const;

            /**
             * This method computes the sift keypoints for the point cloud. The
             * keypoints can be accessed by using the getKeypointCloud() method 
             * of the input fpcfPointCloud object. Before invoking this method,
             * the normals of all points must be computed.
             */
            void detectKeypoints();

        protected:

        private:


    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::fpcfSiftKeypoint(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
        : fpcfDetector(pointCloud)
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::fpcfSiftKeypoint()
        : fpcfDetector()
    {
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::~fpcfSiftKeypoint()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcfSiftKeypoint<PointT, DescriptorT>*
    fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::clone() const
    {
        fpcf::fpcfSiftKeypoint<PointT, DescriptorT> *fpcfSiftKeypoint = new fpcf::fpcfSiftKeypoint<PointT, DescriptorT>();
        return fpcfSiftKeypoint;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::detectKeypoints()
    {
        float min_scale = 0.003f;
        int n_octaves = 8;
        int n_scales_per_octave = 8;
        float min_contrast = 0.0003f;

        // use normals as intensity values
        pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal(new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud<PointT, pcl::PointXYZ>(*pointCloud_->getPointCloud(), *pclPointCloudXYZ);
        pcl::concatenateFields (*pclPointCloudXYZ, *pointCloud_->getNormalsCloud(), *pointNormal);
  
        pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointNormal> sift;
        pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>());
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        sift.setSearchMethod(tree);
        sift.setScales(min_scale, n_octaves, n_scales_per_octave);
        sift.setMinimumContrast(min_contrast);
        sift.setInputCloud(pointNormal);
        sift.compute(*keypoints);
        std::cout << "Found " << keypoints->size() << " sift key points";

        // convert keypoints to PointT format
        pcl::PointCloud<PointT>::Ptr keypointsPointT(new pcl::PointCloud<PointT>());
        pcl::copyPointCloud<pcl::PointNormal, PointT>(*keypoints, *keypointsPointT);
        pointCloud_->setKeypointCloud(keypointsPointT);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSIFTKEYPOINTS_H_