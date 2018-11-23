/**
 * @file    fpcfNormalEstimation.h
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

#ifndef _FPCFNORMALESTIMATION_H_
#define _FPCFNORMALESTIMATION_H_

// STL

// Boost

// PCL
#include <pcl/features/normal_3d_omp.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{

    /**
     * This class estimates point normals for fpcfPointCloudData using the 
     * Principal Component Analysis
     *
     * see paper:
     *   Liang, P. & Todhunter, J. S.
     *   "Representation and recognition of surface shapes in range images: A 
     *   differential geometry approach"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfNormalEstimation
    {
        public:

            /**
             * Constructor that creates a new fpcfNormalEstimation object for a 
             * specified fpcfPointCloud and a specified search radius.
             *
             * @param[in] pointCloud the fpcfPointCloud for the normal computation
             * @param[in] searchRadius the search radius for the radius query 
             *                         to compute the normals
             */
            fpcfNormalEstimation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, double searchRadius);

            /**
             * Constructor that creates a new fpcfNormalEstimation object with a
             * specified search radius.
             *
             * @param[in] searchRadius the search radius for the radius query 
             *                         to compute the normals
             */
            fpcfNormalEstimation(double searchRadius);

            /**
             * Destructor
             */
            virtual ~fpcfNormalEstimation();

            /**
             * Setter for the used search radius of the normal computation. 
             * This radius is used for the neighbor search query and has a 
             * strong influence on the resulting normals.
             *
             * @param[in] searchRadius the used radius of the point neighbors 
             *                         query
             */
            void setSearchRadius(double searchRadius);

            /**
             * Sets the fpcfPointCloud, which will be used during the normal 
             * estimation.
             *
             * @param[in] pointCloud the pointCloud, which will be used
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Getter for the current used search radius during the normal 
             * computation.
             *
             * @return returns the current search radius
             */
            double getSearchReadius();

            /**
             * Getter for the current fpcfPointCloud of the normal estimation
             *
             * @return returns the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

            /**
             * Invokes the normal computation for the point cloud. The 
             * resulting normals can be accessed at the input point cloud by 
             * using the getNormalsCloud() method.
             */
            void estimatePointNormals();

        protected:

        private:
            pcl::NormalEstimationOMP<PointT, pcl::Normal> *normalEstimator_;
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;
            double searchRadius_;

            fpcfNormalEstimation(); // hide default constructor
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::fpcfNormalEstimation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, double searchRadius)
    {
        int nrCores = boost::thread::hardware_concurrency();
        normalEstimator_ = new pcl::NormalEstimationOMP<PointT, pcl::Normal>(nrCores);
        pointCloud_ = pointCloud;
        setSearchRadius(searchRadius);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::fpcfNormalEstimation(double searchRadius)
    {
        int nrCores = boost::thread::hardware_concurrency();
        normalEstimator_ = new pcl::NormalEstimationOMP<PointT, pcl::Normal>(nrCores);
        setSearchRadius(searchRadius);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::~fpcfNormalEstimation()
    {
        delete normalEstimator_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::setSearchRadius(double searchRadius)
    {
        searchRadius_ = searchRadius;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    double
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::getSearchReadius()
    {
        return searchRadius_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::getPointCloud()
    {
        return pointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfNormalEstimation<PointT, DescriptorT>::estimatePointNormals()
    {
        pcl::search::KdTree<PointT>::Ptr kdTree = boost::make_shared<pcl::search::KdTree<PointT>>();
        normalEstimator_->setSearchMethod(kdTree);
        normalEstimator_->setRadiusSearch(searchRadius_);
        normalEstimator_->setInputCloud(pointCloud_->getPointCloud());
        pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
        normalEstimator_->compute(*normalCloud);
        pointCloud_->setNormalsCloud(normalCloud);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFNORMALESTIMATION_H_