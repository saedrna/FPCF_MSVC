/**
 * FPCF - Point Cloud Fusion Framework
 *
 * @file    fpcfFPFHDescriptor.h
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

#ifndef _FPCFFPFHDESCRIPTOR_H_
#define _FPCFFPFHDESCRIPTOR_H_

// STL

// Boost

// PCL
#include <pcl/features/fpfh_omp.h>

// FPCF
#include <fpcf/registration/features/descriptors/fpcfDescriptor.h>

namespace fpcf
{

    /**
     * This class calculates FPFH descriptors for fpcfPointCloudData. The 
     * normals of the fpcfPointCloud need to computed beforehand for the FPFH 
     * computation.
     * 
     * see Paper: 
     *   Rusu, R. B.; Blodow, N. & Beetz, M.
     *   "Fast point feature histograms (FPFH) for 3D registration"
     */
    template <typename PointT>
    class fpcfFPFHDescriptor : public fpcfDescriptor<PointT, pcl::FPFHSignature33>
    {
        public:

            /**
             * Constructor that creates a new fpcfFPFHDescriptor for a 
             * fpcfPointCloud with a specified search radius.
             *
             * @param[in] pointCloud the point cloud used during the descriptor
             *                       computation
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfFPFHDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::FPFHSignature33>::Ptr pointCloud, double searchRadius);

            /**
             * Constructor that creates a new fpcfFPFHDescriptor with a specified search radius.
             *
             * @param[in] searchRadius the search radius during the descriptor computation
             */
            fpcfFPFHDescriptor(double searchRadius);

            /**
             * Destructor
             */
            virtual ~fpcfFPFHDescriptor();

            /**
             * Sets the radius, which will be used to describe the points of 
             * the point cloud.
             *
             * @param[in] searchRadius the search radius of the descriptor 
             *                         computation
             */
            void setSearchRadius(double searchRadius);

            /**
             * Gets the current search radius of the descriptor computation
             *
             * @return returns the current search radius
             */
            double getSearchReadius() const;

            /**
             * Deep copy
             */
            fpcfFPFHDescriptor<PointT>* clone() const;

            /**
             * Computes the point descriptors for all points of the point 
             * cloud. The resulting point descriptors can be accessed by using
             * the getPointDescriptorCloud() method of the input fpcfPointCloud
             * object.
             */
            void calculatePointDescriptors();

            /**
             * Computes the point descriptors for the keypoints of the point 
             * cloud. The resulting point descriptors can be accessed by using
             * the getKeypointDescriptorCloud() method of the input 
             * fpcfPointCloud object.
             */
            void calculateKeypointDescriptors();

        protected:

        private:
            pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> *fpfh_;

            fpcfFPFHDescriptor(); // hide default constructor

            void init();

    };

    template <typename PointT>
    fpcf::fpcfFPFHDescriptor<PointT>::fpcfFPFHDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::FPFHSignature33>::Ptr pointCloud, double searchRadius)
        : fpcfDescriptor(pointCloud)
    {
        int nrCores = boost::thread::hardware_concurrency();
        fpfh_ = new pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>(nrCores);
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfFPFHDescriptor<PointT>::fpcfFPFHDescriptor(double searchRadius)
        : fpcfDescriptor()
    {
        int nrCores = boost::thread::hardware_concurrency();
        fpfh_ = new pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>(nrCores);
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfFPFHDescriptor<PointT>::~fpcfFPFHDescriptor()
    {
        delete fpfh_;
    }

    template <typename PointT>
    void
    fpcf::fpcfFPFHDescriptor<PointT>::setSearchRadius(double searchRadius)
    {
        fpfh_->setRadiusSearch(searchRadius);
    }

    template <typename PointT>
    double 
    fpcf::fpcfFPFHDescriptor<PointT>::getSearchReadius() const
    {
        return fpfh_->getRadiusSearch();
    }

    template <typename PointT>
    fpcfFPFHDescriptor<PointT>*
    fpcf::fpcfFPFHDescriptor<PointT>::clone() const
    {
        double searchRadius = getSearchReadius();
        fpcf::fpcfFPFHDescriptor<PointT> *fpcfFPFHDescriptor = new fpcf::fpcfFPFHDescriptor<PointT>(searchRadius);
        return fpcfFPFHDescriptor;
    }

    template <typename PointT>
    void
    fpcf::fpcfFPFHDescriptor<PointT>::calculatePointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptors_(new pcl::PointCloud<pcl::FPFHSignature33>());
        fpfh_->setInputCloud(pointCloud_->getPointCloud());
        fpfh_->setSearchSurface(NULL);
        fpfh_->compute(*fpfhDescriptors_);
        pointCloud_->setPointDescriptorCloud(fpfhDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfFPFHDescriptor<PointT>::calculateKeypointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptors_(new pcl::PointCloud<pcl::FPFHSignature33>());
        fpfh_->setInputCloud(pointCloud_->getKeypointCloud());
        fpfh_->setSearchSurface(pointCloud_->getPointCloud());
        fpfh_->compute(*fpfhDescriptors_);
        pointCloud_->setKeypointDescriptorCloud(fpfhDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfFPFHDescriptor<PointT>::init()
    {
        pcl::search::KdTree<PointT>::Ptr kdTree = boost::make_shared<pcl::search::KdTree<PointT>>();
        
        fpfh_->setInputCloud(pointCloud_->getPointCloud());
        fpfh_->setInputNormals(pointCloud_->getNormalsCloud());
        fpfh_->setSearchMethod(kdTree);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFFPFHDESCRIPTOR_H_