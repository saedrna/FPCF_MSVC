/**
 * @file    fpcfNarfKeypoint.h
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

#ifndef _FPCFNARFKEYPOINT_H_
#define _FPCFNARFKEYPOINT_H_

// STL

// Boost

// PCL
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>


// FPCF
#include <fpcf/registration/features/detectors/fpcfDetector.h>

namespace fpcf
{

    /**
     * This class computes narf keypoints for fpcfPointCloudData.
     *
     * see paper:
     *   Steder, B.; Rusu, R.; Konolige, K. & Burgard, W.
     *   "Point feature extraction on 3D range scans taking into account object
     *   boundaries"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfNarfKeypoint : public fpcfDetector<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfNarfKeypoint<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfNarfKeypoint detector for a 
             * specified point cloud.
             *
             * @param[in] pointCloud the used point cloud during the 
             *                       comptutation
             */
            fpcfNarfKeypoint(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Constructor that creates a new empty fpcfNarfKeypoint detector
             */
            fpcfNarfKeypoint();

            /**
             * Destructor
             */
            virtual ~fpcfNarfKeypoint();

            /**
             * Getter for the created range image during the keypoint 
             * detection.
             *
             * @result the created range image
             */
            pcl::RangeImage::Ptr getRangeImage();

            /**
             * Getter identified keypoints as indices of the original point.
             *
             * @result the indices of the points of identified as keypoints
             */
            pcl::PointCloud<int>::Ptr getKeypointIndices();

            /**
             * Deep copy
             */
            fpcfNarfKeypoint<PointT, DescriptorT>* clone() const;

            /**
             * This method computes the narf keypoints for the point cloud. The
             * keypoints can be accessed by using the getKeypointCloud() method 
             * of the input fpcfPointCloud object.
             */
            void detectKeypoints();

        protected:

        private:
            pcl::RangeImage::Ptr rangeImage_;
            pcl::PointCloud<int>::Ptr keypointIndices_;

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::fpcfNarfKeypoint(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
        : fpcfDetector(pointCloud)
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::fpcfNarfKeypoint()
        : fpcfDetector()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::~fpcfNarfKeypoint()
    {
    }

    template <typename PointT, typename DescriptorT>
    pcl::RangeImage::Ptr
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::getRangeImage()
    {
        return rangeImage_;
    }
    
    template <typename PointT, typename DescriptorT>
    pcl::PointCloud<int>::Ptr
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::getKeypointIndices()
    {
        return keypointIndices_;
    }

    template <typename PointT, typename DescriptorT>
    fpcfNarfKeypoint<PointT, DescriptorT>*
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::clone() const
    {
        fpcf::fpcfNarfKeypoint<PointT, DescriptorT> *fpcfNarfKeypoint = new fpcf::fpcfNarfKeypoint<PointT, DescriptorT>();
        return fpcfNarfKeypoint;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::detectKeypoints()
    {
        // Parameters
        float angurlarResolution = 0.01f;
        float supportSize = 0.05f;
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        bool setUnseenToMaxRange = true;

        Eigen::Affine3f sensorPose (Eigen::Affine3f::Identity ());
      
        sensorPose = Eigen::Affine3f(Eigen::Translation3f(pointCloud_->getPointCloud()->sensor_origin_[0],
                                                                 pointCloud_->getPointCloud()->sensor_origin_[1],
                                                                 pointCloud_->getPointCloud()->sensor_origin_[2])) *
                            Eigen::Affine3f(pointCloud_->getPointCloud()->sensor_orientation_);
   
        // Create RangeImage from the PointCloud
        float noise_level = 0.0;
        float min_range = 0.0f;
        int border_size = 1;
        rangeImage_.reset(new pcl::RangeImage());
        rangeImage_->createFromPointCloud(*pointCloud_->getPointCloud(), angurlarResolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                         sensorPose, coordinate_frame, noise_level, min_range, border_size);
        rangeImage_->setUnseenToMaxRange();
  
        // Extract NARF keypoints
        pcl::RangeImageBorderExtractor range_image_border_extractor;
        pcl::NarfKeypoint narfKeypointDetector(&range_image_border_extractor);
        narfKeypointDetector.setRangeImage(rangeImage_.get());
        narfKeypointDetector.getParameters().support_size = supportSize;
        narfKeypointDetector.setRadiusSearch(0.05);
        narfKeypointDetector.getParameters().add_points_on_straight_edges = true;
  
        keypointIndices_.reset(new pcl::PointCloud<int>());
        narfKeypointDetector.compute(*keypointIndices_);
        std::cout << "Found " << keypointIndices_->points.size() << " narf key points";

        pcl::PointCloud<PointT>::Ptr keypoints(new pcl::PointCloud<PointT>);
        keypoints->points.resize(keypointIndices_->points.size());
        for (size_t i = 0; i < keypointIndices_->points.size(); ++i)
        {
            keypoints->points[i].getVector3fMap() = rangeImage_->points[keypointIndices_->points[i]].getVector3fMap();
        }
        pointCloud_->setKeypointCloud(keypoints);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFNARFKEYPOINT_H_