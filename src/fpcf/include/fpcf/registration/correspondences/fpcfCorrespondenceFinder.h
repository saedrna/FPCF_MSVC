/**
 * @file    fpcfCorrespondenceFinder.h
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

#ifndef _FPCFCORRESPONDENCEFINDER_H_
#define _FPCFCORRESPONDENCEFINDER_H_

// STL

// Boost

// PCL
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/common/io.h>

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{

    /**
     * This class uses the computed descriptor to create correpondences between
     * the points of a fpcfPointCloudData pair.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfCorrespondenceFinder : public pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT>
    {
        public:

            /**
             * Constructor that creates a new fpcfCorrespondence finder of a 
             * fpcfPointCloudDataPair. The descriptors of the according points 
             * need to be computed beforehand.
             *
             * @param[in] pointCloudPair the pointCloudPair which is used to 
             *                           detect correspondences
             */
            fpcfCorrespondenceFinder(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Destructor
             */
            virtual ~fpcfCorrespondenceFinder();

            /**
             * This method computes the correspondences between the point 
             * clouds. If the point clouds contains no keypoints, then 
             * correspondences are computed using all points of the point 
             * clouds. If the point clouds contain keypoints, then 
             * corresspondences are computed using the keypoints of the point 
             * clouds. The according point descriptors need to be computed 
             * beforehand. The computed correspondences are set at the 
             * fpcfPointCloudDataPair and can be accessed using 
             * getAllPointCorrespondences() or getKeypointCorrespondences().
             *
             * @result returns the number of found correspondences
             */
            int calculateCorrepondences();

            /**
             * This method computes the correspondences between all points of
             * the point clouds. The point descriptors of all points need to be
             * computed beforehand. The computed correspondences are set at the
             * fpcfPointCloudDataPair and can be accessed using 
             * getAllPointCorrespondences().
             *
             * @result returns the number of found correspondences
             */
            int calculateAllPointCorrepondences();

            /**
             * This method computes the correspondences between the keypoints 
             * of the point clouds. The point descriptors of the keypoints need
             * to be computed beforehand. The computed correspondences are set 
             * at the fpcfPointCloudDataPair and can be accessed using 
             * getKeypointCorrespondences().
             *
             * @result returns the number of found correspondences
             */
            int calculateKeypointCorrepondences();

        protected:

        private:
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair_;

            fpcfCorrespondenceFinder(); // hide default constructor

            void determineReciprocalCorrespondences (pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max());

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::fpcfCorrespondenceFinder(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
        : pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT>()
    {
        pointCloudPair_ = pointCloudPair;
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::~fpcfCorrespondenceFinder()
    {

    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::calculateCorrepondences()
    {
        int nrCorrespondences = -1;
        if (pointCloudPair_->getPointCloudSource()->getKeypointDescriptorCloud() && pointCloudPair_->getPointCloudTarget()->getKeypointDescriptorCloud())
        {
            nrCorrespondences = calculateKeypointCorrepondences();
        }
        else
        {
            nrCorrespondences = calculateAllPointCorrepondences();
        }
        return nrCorrespondences;
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::calculateAllPointCorrepondences()
    {
        pcl::DefaultPointRepresentation<DescriptorT>::Ptr pointRepresentation(new pcl::DefaultPointRepresentation<DescriptorT>());
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        setPointRepresentation(pointRepresentation);
        setInputSource(pointCloudPair_->getPointCloudSource()->getPointDescriptorCloud());
        setInputTarget(pointCloudPair_->getPointCloudTarget()->getPointDescriptorCloud());
        determineReciprocalCorrespondences(*correspondences);
        //std::cout << "#CorresFound: " << correspondences->size() << std::endl;
        pointCloudPair_->setAllPointCorrespondences(correspondences);
        return correspondences->size();
    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::calculateKeypointCorrepondences()
    {
        pcl::DefaultPointRepresentation<DescriptorT>::Ptr pointRepresentation(new pcl::DefaultPointRepresentation<DescriptorT>());
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        setPointRepresentation(pointRepresentation);
        setInputSource(pointCloudPair_->getPointCloudSource()->getKeypointDescriptorCloud());
        setInputTarget(pointCloudPair_->getPointCloudTarget()->getKeypointDescriptorCloud());
        determineReciprocalCorrespondences(*correspondences);
        //std::cout << "#CorresFound: " << correspondences->size() << std::endl;
        pointCloudPair_->setKeypointCorrespondences(correspondences);
        return correspondences->size();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT>::determineReciprocalCorrespondences(pcl::Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max ())
    {
        if (!initCompute ())
        {
            return;
        }
  
        typedef typename pcl::traits::fieldList<DescriptorT>::type FieldListSource;
        typedef typename pcl::traits::fieldList<DescriptorT>::type FieldListTarget;
        typedef typename pcl::intersect<FieldListSource, FieldListTarget>::type FieldList;
  
        // setup tree for reciprocal search
        // Set the internal point representation of choice
        if (!initComputeReciprocal())
        {
            return;
        }
        double max_dist_sqr = max_distance * max_distance;

        correspondences.resize (indices_->size());
        std::vector<int> index(2);
        std::vector<float> distance(2);
        std::vector<int> index_reciprocal(2);
        std::vector<float> distance_reciprocal(2);
        pcl::Correspondence corr;
        unsigned int nr_valid_correspondences = 0;
        int target_idx = 0;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx)
        {
            tree_->nearestKSearch(input_->points[*idx], 2, index, distance);
            if (distance[0] > max_dist_sqr || distance[1] / distance[0] < 1.03)
            {
                continue;
            }

            target_idx = index[0];

            tree_reciprocal_->nearestKSearch (target_->points[target_idx], 2, index_reciprocal, distance_reciprocal);
            if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0] || distance_reciprocal[1] / distance_reciprocal[0] < 1.03)
            {
                continue;
            }

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
        correspondences.resize(nr_valid_correspondences);
        deinitCompute();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFCORRESPONDENCEFINDER_H_