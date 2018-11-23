/**
 * @file    fpcfICPPairAlignment.h
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

#ifndef _FPCFICPPAIRALIGNENT_H_
#define _FPCFICPPAIRALIGNENT_H_

// STL

// Boost

// PCL
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{

    /**
     * This class performs a local optimization of the point cloud alignment 
     * between a point cloud pair using the point-to-plane variant of the 
     * iterative closest point algorithm
     *
     * see paper:
     *   Besl, P. J. & McKay, N. D.
     *   "Method for registration of 3-D shapes"
     *
     *   Rusinkiewicz, S. & Levoy, M.
     *   "Efficient variants of the ICP algorithm"   
     */
    template <typename PointT, typename DescriptorT>
    class fpcfICPPairAlignment
    {
        public:
            typedef boost::shared_ptr<fpcfICPPairAlignment<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfICPPairAlignment object for a 
             * specified point cloud pair
             *
             * @param[in] pointCloudPair the point cloud pair which alignment 
             *                           will be optimized
             */
            fpcfICPPairAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Constructor that creates a new empty fpcfICPPairAlignment object
             */
            fpcfICPPairAlignment();

            /**
             * Deep copy constructor
             */
            fpcfICPPairAlignment(const fpcfICPPairAlignment& fpcfICPPairAlignment);

            /**
             * Destructor
             */
            virtual ~fpcfICPPairAlignment();

            /**
             * Getter for the maximum number of iterations of the ICP 
             * algorithm.
             *
             * @return return the number of ICP iterations
             */
            int getIterations() const;

            /**
             * Sets the used point cloud pair, which alignment will be 
             * optimized.
             *
             * @param[in] pointCloudPair the used point cloud pair of the ICP 
             *                           optimization
             */
            void setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Sets the maximum number of iterations of the ICP algorithm.
             *
             * @param[in] iterations the maximum number of ICP iterations
             */
            void setIterations(int iterations);

            /**
             * Performs a local optimization of the alignment of the point 
             * cloud pair. The resulting transformation matrix have to be
             * applied to the source point cloud in order to align it to the 
             * target point cloud of the point cloud pair.
             *
             * @param[out] transformation the transformation matrix to align 
             *                            the source point cloud to the target 
             *                            point cloud.
             */
            void calculateTransformation(Eigen::Matrix4f &transformation);

        protected:

        private:
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair_;
            pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp_;
            int iterations_;

            void createPointNormals(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, typename pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals);
            void init();

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::fpcfICPPairAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        pointCloudPair_ = pointCloudPair;
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::fpcfICPPairAlignment()
    {
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::fpcfICPPairAlignment(const fpcfICPPairAlignment& fpcfICPPairAlignment)
    {
        init();
        setIterations(fpcfICPPairAlignment.getIterations());
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::~fpcfICPPairAlignment()
    {

    }

    template <typename PointT, typename DescriptorT>
    int
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::getIterations() const
    {
        return iterations_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        pointCloudPair_ = pointCloudPair;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::setIterations(int iterations)
    {
        iterations_ = iterations;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::calculateTransformation(Eigen::Matrix4f &transformation)
    {
        typename pcl::PointCloud<pcl::PointNormal>::Ptr sourcePointNormals(new pcl::PointCloud<pcl::PointNormal>());
        typename pcl::PointCloud<pcl::PointNormal>::Ptr targetPointNormals(new pcl::PointCloud<pcl::PointNormal>());
        createPointNormals(pointCloudPair_->getPointCloudSource(), sourcePointNormals);
        createPointNormals(pointCloudPair_->getPointCloudTarget(), targetPointNormals);
        icp_.setInputSource(sourcePointNormals);
        icp_.setInputTarget(targetPointNormals);
        icp_.setMaximumIterations(iterations_);
        std::cout << "begin icp with #iters: " << iterations_ << std::endl;
        pcl::PointCloud<pcl::PointNormal> final;
        icp_.align(final);
        transformation = icp_.getFinalTransformation();

        std::cout << "icp score: " << icp_.getFitnessScore() << std::endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::createPointNormals(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, typename pcl::PointCloud<pcl::PointNormal>::Ptr pointNormals)
    {
        for (int i = 0; i < fpcfPointCloud->getMostDownsampledPointCloud()->getPointCloud()->size(); i++)
        {
            PointT point = fpcfPointCloud->getMostDownsampledPointCloud()->getPointCloud()->at(i);
            pcl::Normal normal = fpcfPointCloud->getMostDownsampledPointCloud()->getNormalsCloud()->at(i);
            pcl::PointNormal *pointNormal = new pcl::PointNormal();
            pointNormal->x = point.x;
            pointNormal->y = point.y;
            pointNormal->z = point.z;
            pointNormal->normal_x = normal.normal_x;
            pointNormal->normal_y = normal.normal_y;
            pointNormal->normal_z = normal.normal_z;
            pointNormals->push_back(*pointNormal);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::init()
    {
        const int DEFAULT_ITERATIONS = 25;
        const float DEFAULT_MAX_CORRESPONDENCE_DISTANCE = 0.05;
        setIterations(DEFAULT_ITERATIONS);
        icp_.setMaxCorrespondenceDistance(DEFAULT_MAX_CORRESPONDENCE_DISTANCE);
        icp_.setUseReciprocalCorrespondences(true);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFICPPAIRALIGNENT_H_