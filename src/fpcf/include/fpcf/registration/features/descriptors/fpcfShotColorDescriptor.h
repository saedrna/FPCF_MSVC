/**
 * @file    fpcfShotColorDescriptor.h
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

#ifndef _FPCFSHOTCOLORDESCRIPTOR_H_
#define _FPCFSHOTCOLORDESCRIPTOR_H_

// STL

// Boost

// PCL
#include <pcl/features/shot_omp.h>

// FPCF
#include <fpcf/registration/features/descriptors/fpcfDescriptor.h>

namespace fpcf
{
    /**
     * This class calculates SHOT descriptors for fpcfPointCloudData. The 
     * normals of the fpcfPointCloud need to computed beforehand for the SHOT
     * computation.
     * 
     * see Paper: 
     *   Tombari, F.; Salti, S. & Di Stefano, L.
     *   "A combined texture-shape descriptor for enhanced 3D feature matching"
     */
    template <typename PointT>
    class fpcfShotColorDescriptor : public fpcfDescriptor<PointT, pcl::SHOT1344>
    {
        public:

            /**
             * Constructor that creates a new fpcfShotColorDescriptor for a 
             * fpcfPointCloud with a specified search radius.
             *
             * @param[in] pointCloud the point cloud used during the descriptor
             *                       computation
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfShotColorDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::SHOT1344>::Ptr pointCloud, double searchRadius);

            /**
             * Constructor that creates a new fpcfShotColorDescriptor with a 
             * specified search radius.
             *
             * @param[in] searchRadius the search radius during the descriptor 
             *                         computation
             */
            fpcfShotColorDescriptor(double searchRadius);

            /**
             * Destructor
             */
            virtual ~fpcfShotColorDescriptor();

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
            fpcfShotColorDescriptor<PointT>* clone() const;

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
            double searchRadius_;

    };

    template <typename PointT>
    fpcf::fpcfShotColorDescriptor<PointT>::fpcfShotColorDescriptor(typename fpcf::fpcfPointCloudData<PointT, pcl::SHOT1344>::Ptr pointCloud, double searchRadius)
        : fpcfDescriptor(pointCloud)
    {
        setSearchRadius(searchRadius);
    }

    template <typename PointT>
    fpcf::fpcfShotColorDescriptor<PointT>::fpcfShotColorDescriptor(double searchRadius)
        : fpcfDescriptor()
    {
        setSearchRadius(searchRadius);
    }
    
    template <typename PointT>
    fpcf::fpcfShotColorDescriptor<PointT>::~fpcfShotColorDescriptor()
    {
    }

    template <typename PointT>
    void
    fpcf::fpcfShotColorDescriptor<PointT>::setSearchRadius(double searchRadius)
    {
        searchRadius_ = searchRadius;
    }

    template <typename PointT>
    double 
    fpcf::fpcfShotColorDescriptor<PointT>::getSearchReadius() const
    {
        return searchRadius_;
    }

    template <typename PointT>
    fpcfShotColorDescriptor<PointT>*
    fpcf::fpcfShotColorDescriptor<PointT>::clone() const
    {
        double searchRadius = getSearchReadius();
        fpcf::fpcfShotColorDescriptor<PointT> *fpcfShotColorDescriptor = new fpcf::fpcfShotColorDescriptor<PointT>(searchRadius);
        return fpcfShotColorDescriptor;
    }

    template <typename PointT>
    void
    fpcf::fpcfShotColorDescriptor<PointT>::calculatePointDescriptors()
    {
        pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344> ShotColor;
        pcl::PointCloud<pcl::SHOT1344>::Ptr shotDescriptors_(new pcl::PointCloud<pcl::SHOT1344>());

        ShotColor.nr_shape_bins_ = 16;

        ShotColor.setRadiusSearch(searchRadius_);
        ShotColor.setInputCloud(pointCloud_->getPointCloud());
        ShotColor.setInputNormals(pointCloud_->getNormalsCloud());
        ShotColor.setSearchSurface(NULL);
        ShotColor.compute(*shotDescriptors_);
        // refactor this here:
        std::vector<int> goodIndices;
        for (int i = 0; i < shotDescriptors_->size(); i++)
        {
            if (!pcl_isnan(shotDescriptors_->at(i).rf[0]))
            {
                goodIndices.push_back(i);
            }
        }
        pcl::PointCloud<PointT>::Ptr newPointCloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::SHOT1344>::Ptr newShotDescriptors_(new pcl::PointCloud<pcl::SHOT1344>());
        pcl::copyPointCloud(*pointCloud_->getPointCloud(), goodIndices, *newPointCloud);
        pcl::copyPointCloud(*shotDescriptors_, goodIndices, *newShotDescriptors_);

        pointCloud_->setPointCloud(newPointCloud);
        pointCloud_->setPointDescriptorCloud(newShotDescriptors_);
    }

    template <typename PointT>
    void
    fpcf::fpcfShotColorDescriptor<PointT>::calculateKeypointDescriptors()
    {
        pcl::SHOTColorEstimationOMP<PointT, pcl::Normal, pcl::SHOT1344> ShotColor;
        pcl::PointCloud<pcl::SHOT1344>::Ptr shotDescriptors_(new pcl::PointCloud<pcl::SHOT1344>());
        ShotColor.setRadiusSearch(searchRadius_);
        ShotColor.setInputCloud(pointCloud_->getKeypointCloud());
        ShotColor.setSearchSurface(pointCloud_->getPointCloud());
        ShotColor.setInputNormals(pointCloud_->getNormalsCloud());
        ShotColor.compute(*shotDescriptors_);
        // refactor this here:
        std::vector<int> goodIndices;
        for (int i = 0; i < shotDescriptors_->size(); i++)
        {
            if (!pcl_isnan(shotDescriptors_->at(i).rf[0]))
            {
                goodIndices.push_back(i);
            }
        }
        pcl::PointCloud<PointT>::Ptr newKeypointCloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::SHOT1344>::Ptr newShotDescriptors_(new pcl::PointCloud<pcl::SHOT1344>());
        pcl::copyPointCloud(*pointCloud_->getKeypointCloud(), goodIndices, *newKeypointCloud);
        pcl::copyPointCloud(*shotDescriptors_, goodIndices, *newShotDescriptors_);

        pointCloud_->setKeypointCloud(newKeypointCloud);
        pointCloud_->setKeypointDescriptorCloud(newShotDescriptors_);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSHOTCOLORDESCRIPTOR_H_