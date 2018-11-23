/**
 * @file    fpcfCorrespondenceRejector.h
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

#ifndef _FPCFCORRESPONDENCEREJECTOR_H_
#define _FPCFCORRESPONDENCEREJECTOR_H_

// STL

// Boost

// PCL
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


#include <pcl/registration/correspondence_rejection_poly.h>


// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{
    /**
     * This class rejects wrong correpondences between a fpcfPointCloudData 
     * pair using a combination of a polygon rejector and a Ransac rejector. 
     * The fpcfCorrespondenceFinder must be executed beforehand to set the 
     * according point correspondences.
     *
     * see Papers: 
     *   Buch, A. G.; Kraft, D.; Kaemaeraeinen, J.-K.; Petersen, H. G. & 
     *   Krueger, N.
     *   "Pose estimation using local structure-specific shape and appearance 
     *   context"
     *
     *   Fischler, M. A. & Bolles, R. C.
     *   "Random sample consensus: a paradigm for model fitting with 
     *   applications to image analysis and automated cartography"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfCorrespondenceRejector
    {
        public:
            typedef boost::shared_ptr<fpcfCorrespondenceRejector<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfCorrespondences rejector for a
             * fpcfPointCloudDataPair.
             *
             * @param[in] pointCloudPair the pointClourPair with correspondences
             */
            fpcfCorrespondenceRejector(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Constructor that creates a new empty fpcfCorrespondences rejector
             */
            fpcfCorrespondenceRejector();

            /**
             * Deep copy constructor
             */
            fpcfCorrespondenceRejector(const fpcfCorrespondenceRejector& fpcfCorrespondenceRejector);

            /**
             * Destructor
             */
            virtual ~fpcfCorrespondenceRejector();

            /**
             * Getter for the used number of Ransac iteration to find the 
             * correspondence inliers.
             *
             * @return return the number of Ransac interations
             */
            int getRansacIterations() const;

            /**
             * Getter for the current PointCloudPair of the correspondenceRejector
             *
             * @return returns the current PointCloudPair
             */
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr getPointCloudPair();

            /**
             * Setter for the number of Ransac iteration to find the 
             * correspondence inliers.
             *
             * @param[in] ransacIterations the number of Ransac interations
             */
            void setRansacIterations(int ransacIterations);

            /**
             * Setter for the used PointCloudPair of the 
             * correspondenceRejector. The pointCloudPair should need to have 
             * correspondences.
             *
             * @param[in] fpcfPointCloudPair the PointCloudPair
             */
            void setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);

            /**
             * Rejects the wrong correspondences between the point clouds of 
             * the point cloud pair. If keypoint correspondences were 
             * determined, then these correspondences are used during the 
             * rejection. Otherwise, the correspondences between all points are
             * used. The correspondences need to be determined from the 
             * fpcfCorrespondenceFinder beforehand.
             *
             * @return returns the number of remaining correspondences
             */
            int rejectCorrespondences();

            /**
             * Rejects the wrong correspondences between all points of the 
             * point cloud pair. All point correspondences need to be 
             * determined from the fpcfCorrespondenceFinder beforehand.
             *
             * @return returns the number of remaining correspondences
             */
            int rejectAllPointCorrespondences();

            /**
             * Rejects the wrong correspondences between the keypoints of the 
             * point cloud pair. The keypoint correspondences need to be 
             * determined from the fpcfCorrespondenceFinder beforehand.
             *
             * @return returns the number of remaining correspondences
             */
            int rejectKeypointCorrespondences();

        protected:

        private:
            static const int DEFAULT_RANSAC_ITERATIONS = 100;

            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair_;
            int ransacIterations_;

            void init();
            int rejectCorrespondences(typename pcl::PointCloud<PointT>::Ptr inputCloud, typename pcl::PointCloud<PointT>::Ptr targetCloud, pcl::CorrespondencesPtr currentCorrespondences, pcl::CorrespondencesPtr newCorrepondences);

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::fpcfCorrespondenceRejector(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        pointCloudPair_ = pointCloudPair;
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::fpcfCorrespondenceRejector()
    {
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::fpcfCorrespondenceRejector(const fpcfCorrespondenceRejector& fpcfCorrespondenceRejector)
    {
        init();
        setRansacIterations(fpcfCorrespondenceRejector.getRansacIterations());
    }
    
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::~fpcfCorrespondenceRejector()
    {
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::getRansacIterations() const
    {
        return ransacIterations_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr 
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::getPointCloudPair()
    {
        return pointCloudPair_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::setRansacIterations(int ransacIterations)
    {
        ransacIterations_ = ransacIterations;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        pointCloudPair_ = fpcfPointCloudPair;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::init()
    {
        setRansacIterations(DEFAULT_RANSAC_ITERATIONS);
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::rejectCorrespondences()
    {
        int nrCorrespondences = -1;
        if (pointCloudPair_->getKeypointCorrespondences())
        {
            nrCorrespondences = rejectKeypointCorrespondences();
        }
        else
        {
            nrCorrespondences = rejectAllPointCorrespondences();
        }
        return nrCorrespondences;
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::rejectAllPointCorrespondences()
    {
        int nrCorrespondences;
        pcl::CorrespondencesPtr newCorrespondences(new pcl::Correspondences());
        nrCorrespondences = rejectCorrespondences(
            pointCloudPair_->getPointCloudSource()->getPointCloud(),
            pointCloudPair_->getPointCloudTarget()->getPointCloud(), 
            pointCloudPair_->getAllPointCorrespondences(),
            newCorrespondences
            );
        pointCloudPair_->setAllPointCorrespondences(newCorrespondences);
        return nrCorrespondences;
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::rejectKeypointCorrespondences()
    {
        int nrCorrespondences;
        pcl::CorrespondencesPtr newCorrespondences(new pcl::Correspondences());
        nrCorrespondences = rejectCorrespondences(
            pointCloudPair_->getPointCloudSource()->getKeypointCloud(),
            pointCloudPair_->getPointCloudTarget()->getKeypointCloud(), 
            pointCloudPair_->getKeypointCorrespondences(),
            newCorrespondences
            );
        pointCloudPair_->setKeypointCorrespondences(newCorrespondences);
        return nrCorrespondences;
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::rejectCorrespondences(typename pcl::PointCloud<PointT>::Ptr inputCloud, typename pcl::PointCloud<PointT>::Ptr targetCloud, pcl::CorrespondencesPtr correspondences, pcl::CorrespondencesPtr newCorrespondences)
    {
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransacRejector;
        pcl::registration::CorrespondenceRejectorPoly<PointT, PointT> polyRejector;

        pcl::CorrespondencesPtr polyCorrespondences(new pcl::Correspondences());
        polyRejector.setInputSource(inputCloud);
        polyRejector.setInputTarget(targetCloud);
        polyRejector.getRemainingCorrespondences(*correspondences, *polyCorrespondences);

        ransacRejector.setInputSource(inputCloud);
        ransacRejector.setInputTarget(targetCloud);
        ransacRejector.setInlierThreshold(0.25);
        ransacRejector.setMaximumIterations(ransacIterations_);
        ransacRejector.setRefineModel(true);
        ransacRejector.getRemainingCorrespondences(*polyCorrespondences, *newCorrespondences);
        //std::cout << "after correspondence rejector #corres: " << newCorrespondences->size() << std::endl;

        return newCorrespondences->size();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFCORRESPONDENCEREJECTOR_H_