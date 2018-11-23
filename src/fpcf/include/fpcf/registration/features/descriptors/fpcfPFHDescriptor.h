/**
 * @file    fpcfPFHDescriptor.h
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

#ifndef _FPCFPFHDESCRIPTOR_H_
#define _FPCFPFHDESCRIPTOR_H_

// STL

// Boost

// PCL
#include <pcl/features/pfh.h>

// FPCF
#include <fpcf/registration/features/descriptors/fpcfDescriptor.h>

namespace fpcf
{

    /**
     * This class calculates PFH descriptors for fpcfPointCloudData. The normals
     * of the fpcfPointCloud need to computed beforehand for the PFH 
     * computation.
     * 
     * see Paper: 
     *   Rusu, R.; Blodow, N.; Marton, Z. & Beetz, M.
     *   "Aligning point cloud views using persistent feature histograms"
     */
    template <typename PointT>
    class fpcfPFHDescriptor : public fpcfDescriptor<PointT, pcl::PFHSignature125>
    {
        public:

            /**
             * Constructor that creates a new fpcfPFHDescriptor for a 
             * fpcfPointCloud with a specified search radius.
             *
             * @param[in] pointCloud the point cloud used during the descriptor
             *                       computation
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfPFHDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::PFHSignature125>::Ptr pointCloud, double searchRadius);

            /**
             * Constructor that creates a new fpcfPFHDescriptor with a specified
             * search radius.
             *
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfPFHDescriptor(double searchRadius);

            /**
             * Destructor
             */
            virtual ~fpcfPFHDescriptor();

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
            fpcfPFHDescriptor<PointT>* clone() const;

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
            pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh_;

            fpcfPFHDescriptor(); // hide default constructor

            void init();

    };

    template <typename PointT>
    fpcf::fpcfPFHDescriptor<PointT>::fpcfPFHDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::PFHSignature125>::Ptr pointCloud, double searchRadius)
        : fpcfDescriptor(pointCloud)
    {
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfPFHDescriptor<PointT>::fpcfPFHDescriptor(double searchRadius)
        : fpcfDescriptor()
    {
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfPFHDescriptor<PointT>::~fpcfPFHDescriptor()
    {
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHDescriptor<PointT>::setSearchRadius(double searchRadius)
    {
        pfh_.setRadiusSearch(searchRadius);
    }

    template <typename PointT>
    double 
    fpcf::fpcfPFHDescriptor<PointT>::getSearchReadius() const
    {
        return pfh_.getRadiusSearch();
    }

    template <typename PointT>
    fpcfPFHDescriptor<PointT>*
    fpcf::fpcfPFHDescriptor<PointT>::clone() const
    {
        double searchRadius = getSearchReadius();
        fpcf::fpcfPFHDescriptor<PointT> *fpcfPFHDescriptor = new fpcf::fpcfPFHDescriptor<PointT>(searchRadius);
        return fpcfPFHDescriptor;
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHDescriptor<PointT>::calculatePointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhDescriptors_(new pcl::PointCloud<pcl::PFHSignature125>());
        pfh_.setInputCloud(pointCloud_->getPointCloud());
        pfh_.setSearchSurface(NULL);
        pfh_.compute(*pfhDescriptors_);
        pointCloud_->setPointDescriptorCloud(pfhDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHDescriptor<PointT>::calculateKeypointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhDescriptors_(new pcl::PointCloud<pcl::PFHSignature125>());
        pfh_.setInputCloud(pointCloud_->getKeypointCloud());
        pfh_.setSearchSurface(pointCloud_->getPointCloud());
        pfh_.compute(*pfhDescriptors_);
        pointCloud_->setKeypointDescriptorCloud(pfhDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHDescriptor<PointT>::init()
    {
        pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());
        pfh_.setInputNormals(pointCloud_->getNormalsCloud());
        pfh_.setSearchMethod(kdTree);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPFHDESCRIPTOR_H_