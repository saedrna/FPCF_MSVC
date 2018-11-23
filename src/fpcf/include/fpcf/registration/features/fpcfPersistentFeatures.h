/**
 * @file    fpcfPersistentFeatures.h
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

#ifndef _FPCFPERSISTENTFEATURES_H_
#define _FPCFPERSISTENTFEATURES_H_

// STL

// Boost

// PCL
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/fpfh_omp.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{

    /**
     * This class calculates persistant features for fpcfPointCloudData.
     * This class uses FPFH descriptors for persistant feature detection.
     * The normals of the input point cloud need to be computed beforehand.
     *
     * see Paper: 
     *   Rusu, R. B.; Blodow, N. & Beetz, M.
     *   "Fast point feature histograms (FPFH) for 3D registration"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPersistentFeatures
    {
        public:

            /**
             * Constructor that creates a new fpcfPersistentFeatures object for 
             * a specific point cloud.
             *
             * @param[in] pointCloud the point cloud for the peristent features computation.
             */
            fpcfPersistentFeatures(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Constructor that creates a new empty fpcfPersistentFeatures object
             */
            fpcfPersistentFeatures();

            /**
             * Destructor
             */
            virtual ~fpcfPersistentFeatures();

            /**
             * Sets the fpcfPointCloud, which will be used during persistent 
             * features computation.
             *
             * @param[in] pointCloud the pointCloud, which will be used
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr);

            /**
             * Getter for the current fpcfPointCloud of the persistent features 
             * computation
             *
             * @return returns the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

            /**
             * Computes the peristent features for the input point cloud and 
             * directly removes all points of the point cloud, which are non 
             * persistent. The normals of the input point cloud need to be 
             * computed beforehand.
             */
            void calculatePersistentFeatures();

        protected:

        private:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud_;
            pcl::MultiscaleFeaturePersistence<PointT, pcl::FPFHSignature33> persisFeatures_;

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::fpcfPersistentFeatures(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        setPointCloud(pointCloud);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::fpcfPersistentFeatures()
    {
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::~fpcfPersistentFeatures()
    {
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pointCloud_ = pointCloud;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::getPointCloud()
    {
        return pointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPersistentFeatures<PointT, DescriptorT>::calculatePersistentFeatures()
    {
        int nrCores = boost::thread::hardware_concurrency();
        pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr pfh(new pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>(nrCores));
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
        typename pcl::PointCloud<PointT>::Ptr newPointCloud(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<DescriptorT>::Ptr newPointDescriptorCloud(new pcl::PointCloud<DescriptorT>());
        typename pcl::PointCloud<pcl::Normal>::Ptr newNormalsCloud(new pcl::PointCloud<pcl::Normal>());
        pfh->setInputCloud(pointCloud_->getPointCloud());
        pfh->setInputNormals(pointCloud_->getNormalsCloud());

        
        pcl::DefaultPointRepresentation<pcl::FPFHSignature33>::Ptr pointRepresentation(new pcl::DefaultPointRepresentation<pcl::FPFHSignature33>());
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr featureCloud(new pcl::PointCloud<pcl::FPFHSignature33>());
        boost::shared_ptr<std::vector<int>> persisFeatureIndices(new std::vector<int>());

        std::vector<float> scaleVector;
        scaleVector.push_back(0.0325f);
        scaleVector.push_back(0.0350f);
        scaleVector.push_back(0.0375f);

        persisFeatures_.setInputCloud(pointCloud_->getPointCloud());
        persisFeatures_.setPointRepresentation(pointRepresentation);
        persisFeatures_.setFeatureEstimator(pfh);
        persisFeatures_.setScalesVector(scaleVector);
        persisFeatures_.setAlpha(0.75f);
        persisFeatures_.setDistanceMetric(pcl::NormType::L1); 
        persisFeatures_.determinePersistentFeatures(*featureCloud, persisFeatureIndices);

        std::cout << "#points after persistent features: " << featureCloud->size() << std::endl;
        pcl::copyPointCloud(*pointCloud_->getPointCloud(), *persisFeatureIndices, *newPointCloud);
        pointCloud_->setPointCloud(newPointCloud);
        if (pointCloud_->getPointDescriptorCloud())
        {
            pcl::copyPointCloud(*pointCloud_->getPointDescriptorCloud(), *persisFeatureIndices, *newPointDescriptorCloud);
            pointCloud_->setPointDescriptorCloud(newPointDescriptorCloud);
        }
        pcl::copyPointCloud(*pointCloud_->getNormalsCloud(), *persisFeatureIndices, *newNormalsCloud);
        pointCloud_->setNormalsCloud(newNormalsCloud);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPERSISTENTFEATURES_H_