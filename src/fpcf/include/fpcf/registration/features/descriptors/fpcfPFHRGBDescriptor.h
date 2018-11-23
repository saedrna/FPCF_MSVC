/**
 * @file    fpcfPFHRGBDescriptor.h
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

#ifndef _FPCFPFHRGBDESCRIPTOR_H_
#define _FPCFPFHRGBDESCRIPTOR_H_

// STL

// Boost

// PCL
#include <pcl/features/pfhrgb.h>

// FPCF
#include <fpcf/registration/features/descriptors/fpcfDescriptor.h>

namespace fpcf
{

    /**
     * This class calculates PFHRGB descriptors for fpcfPointCloudData. The 
     * normals if the fpcfPointCloud need to computed beforehand for the PFH 
     * computation.
     */
    template <typename PointT>
    class fpcfPFHRGBDescriptor : public fpcfDescriptor<PointT, pcl::PFHRGBSignature250>
    {
        public:

            /**
             * Constructor that creates a new fpcfPFHRGBDescriptor for a 
             * fpcfPointCloud with a specified search radius.
             *
             * @param[in] pointCloud the point cloud used during the descriptor
             *                       computation
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfPFHRGBDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::PFHRGBSignature250>::Ptr pointCloud, double searchRadius);

            /**
             * Constructor that creates a new fpcfPFHRGBDescriptor with a 
             * specified search radius.
             *
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfPFHRGBDescriptor(double searchRadius);

            /**
             * Destructor
             */
            virtual ~fpcfPFHRGBDescriptor();

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
            fpcfPFHRGBDescriptor<PointT>* clone() const;

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
            pcl::PFHRGBEstimation<PointT, pcl::Normal, pcl::PFHRGBSignature250> pfhRGB_;

            fpcfPFHRGBDescriptor(); // hide default constructor

            void init();

    };

    template <typename PointT>
    fpcf::fpcfPFHRGBDescriptor<PointT>::fpcfPFHRGBDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::PFHRGBSignature250>::Ptr pointCloud, double searchRadius)
        : fpcfDescriptor(pointCloud)
    {
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfPFHRGBDescriptor<PointT>::fpcfPFHRGBDescriptor(double searchRadius)
        : fpcfDescriptor()
    {
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfPFHRGBDescriptor<PointT>::~fpcfPFHRGBDescriptor()
    {
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHRGBDescriptor<PointT>::setSearchRadius(double searchRadius)
    {
        pfhRGB_.setRadiusSearch(searchRadius);
    }

    template <typename PointT>
    double 
    fpcf::fpcfPFHRGBDescriptor<PointT>::getSearchReadius() const
    {
        return pfhRGB_.getRadiusSearch();
    }

    template <typename PointT>
    fpcfPFHRGBDescriptor<PointT>*
    fpcf::fpcfPFHRGBDescriptor<PointT>::clone() const
    {
        double searchRadius = getSearchReadius();
        fpcf::fpcfPFHRGBDescriptor<PointT> *fpcfPFHRGBDescriptor = new fpcf::fpcfPFHRGBDescriptor<PointT>(searchRadius);
        return fpcfPFHRGBDescriptor;
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHRGBDescriptor<PointT>::calculatePointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhRGBDescriptors_(new pcl::PointCloud<pcl::PFHRGBSignature250>());
        pfhRGB_.setInputCloud(pointCloud_->getPointCloud());
        pfhRGB_.setSearchSurface(NULL);
        pfhRGB_.compute(*pfhRGBDescriptors_);
        pointCloud_->setPointDescriptorCloud(pfhRGBDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHRGBDescriptor<PointT>::calculateKeypointDescriptors()
    {
        init();
        pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pfhRGBDescriptors_(new pcl::PointCloud<pcl::PFHRGBSignature250>());
        pfhRGB_.setInputCloud(pointCloud_->getKeypointCloud());
        pfhRGB_.setSearchSurface(pointCloud_->getPointCloud());
        pfhRGB_.compute(*pfhRGBDescriptors_);
        pointCloud_->setKeypointDescriptorCloud(pfhRGBDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfPFHRGBDescriptor<PointT>::init()
    {
        pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>());
        pfhRGB_.setInputNormals(pointCloud_->getNormalsCloud());
        pfhRGB_.setSearchMethod(kdTree);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPFHRGBDESCRIPTOR_H_