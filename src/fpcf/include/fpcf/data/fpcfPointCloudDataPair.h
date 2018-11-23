/**
 * @file    fpcfPointCloudDataPair.h
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

#ifndef _FPCFPOINTCLOUDDATAPAIR_H_
#define _FPCFPOINTCLOUDDATAPAIR_H_

// STL

// Boost

// PCL
#include <pcl/correspondence.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * This class contains a point cloud pair for the pairwise registration. 
     * One point cloud represents the source and one point cloud the target
     * point cloud. This class contains the correspondences between the 
     * point clouds and the computed transformation to align the source point
     * cloud to the target point cloud.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudDataPair
    {
        public:
            typedef boost::shared_ptr<fpcfPointCloudDataPair<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new empty point cloud pair
             */
            fpcfPointCloudDataPair();

            /**
             * Constructor that creates a new point cloud pair from a source 
             * and a target fpcfPointCloud
             *
             * @param[in] pointCloudSource the source point cloud of the 
             *                             pairwise alignment
             * @param[in] pointCloudTarget the target point cloud, its 
             *                             coordinate system will be used to 
             *                             align the source point cloud
             */
            fpcfPointCloudDataPair(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget);

            /**
             * Deep copy constructor
             */
            fpcfPointCloudDataPair(const fpcfPointCloudDataPair& fpcfPointCloudDataPair);

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudDataPair();

            /**
             * Getter for the source point cloud
             *
             * @return returns the source point cloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloudSource() const;

            /**
             * Getter for the target point cloud
             *
             * @return returns the target point cloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloudTarget() const;

            /**
             * Getter for the point correspondence between all points of the 
             * point clouds. This is null, if the fpcfCorrespondenceFinder 
             * wasn't executed to find correspondences between all points.
             *
             * @return returns the correspondences between all points of the 
             *         point clouds
             */
            pcl::CorrespondencesPtr getAllPointCorrespondences() const;

            /**
             * Getter for the point correspondence between the keypoints of
             * the point clouds. This is null, if the fpcfCorrespondenceFinder 
             * wasn't executed to find correspondences between the keypoints.
             *
             * @return returns the correspondences between the keypoints of the 
             *         point clouds
             */
            pcl::CorrespondencesPtr getKeypointCorrespondences() const;

            /**
             * Getter for the current set transformation to align the point 
             * cloud source to the point cloud target of the point cloud pair.
             * This method can be used to access the resulting transformation 
             * matrix of a registration computation. If no registration was 
             * performed beforehand, the matrix doesn't contain meaningful 
             * values.
             *
             * @result the current transformation matrix to align the point 
             *         cloud source with the point cloud target
             */
            Eigen::Matrix4f getTransformation() const;

            /**
             * Sets a new fpcfPointCloud as source point cloud
             *
             * @param[in] pointCloudSource the new source point cloud
             */
            void setPointCloudSource(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource);

            /**
             * Sets a new fpcfPointCloud as target point cloud
             *
             * @param[in] pointCloudTarget the new source point cloud
             */
            void setPointCloudTarget(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget);

            /**
             * Sets the new point correspondences between all points of the 
             * point clouds. This is used by the fpcfCorrespondenceFinder to
             * set correspondences between all points of the point clouds.
             *
             * @param[in] allPointCorrespondences the new correspondences
             *                                    between all points of the 
             *                                    point clouds
             */
            void setAllPointCorrespondences(pcl::CorrespondencesPtr allPointCorrespondences);

            /**
             * Sets the new point correspondences between the keyoints of the
             * point clouds. This is used by the fpcfCorrespondenceFinder to 
             * find correspondences between the keypoints of the point clouds.
             *
             * @param[in] keypointCorrespondences the new correspondences 
             *                                    between the keypoints of the 
             *                                    point clouds
             */
            void setKeypointCorrespondences(pcl::CorrespondencesPtr keypointCorrespondences);

            /**
             * Sets the transformation, which aligns the source point cloud to 
             * the target point cloud. This is used by the 
             * fpcfPointCloudPairTransformation.
             *
             * @param[in] transformation the new transformation to align the 
             *                           source point cloud to the target point
             *                           cloud
             */
            void setTransformation(Eigen::Matrix4f &transformation);

        protected:
            
        private:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource_;
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget_;
            pcl::CorrespondencesPtr allPointCorrespondences_;
            pcl::CorrespondencesPtr keypointCorrespondences_;

            Eigen::Matrix4f transformation_;
    };

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudDataPair<PointT, DescriptorT>::fpcfPointCloudDataPair()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudDataPair<PointT, DescriptorT>::fpcfPointCloudDataPair(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget)
    {
        setPointCloudSource(pointCloudSource);
        setPointCloudTarget(pointCloudTarget);
    }

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudDataPair<PointT, DescriptorT>::fpcfPointCloudDataPair(const fpcfPointCloudDataPair& fpcfPointCloudDataPair)
    {
        if (fpcfPointCloudDataPair.getPointCloudSource())
        {
            fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSourceCopy(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(*fpcfPointCloudDataPair.getPointCloudSource()));
            setPointCloudSource(pointCloudSourceCopy);
        }
        if (fpcfPointCloudDataPair.getPointCloudTarget())
        {
            fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTargetCopy(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(*fpcfPointCloudDataPair.getPointCloudTarget()));
            setPointCloudTarget(pointCloudTargetCopy);
        }
        if (fpcfPointCloudDataPair.getAllPointCorrespondences())
        {
            pcl::CorrespondencesPtr allPointCorrespondencesCopy(new pcl::Correspondences(*fpcfPointCloudDataPair.getAllPointCorrespondences()));
            setAllPointCorrespondences(allPointCorrespondencesCopy);
        }
        if (fpcfPointCloudDataPair.getKeypointCorrespondences())
        {
            pcl::CorrespondencesPtr keypointCorrespondencesCopy(new pcl::Correspondences(*fpcfPointCloudDataPair.getKeypointCorrespondences()));
            setKeypointCorrespondences(keypointCorrespondencesCopy);
        }
        setTransformation(fpcfPointCloudDataPair.getTransformation());
    }
                
    template <typename PointT, typename DescriptorT>
    fpcfPointCloudDataPair<PointT, DescriptorT>::~fpcfPointCloudDataPair()
    {
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfPointCloudDataPair<PointT, DescriptorT>::getPointCloudSource() const
    {
        return pointCloudSource_;
    }
            
    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfPointCloudDataPair<PointT, DescriptorT>::getPointCloudTarget() const
    {
        return pointCloudTarget_;
    }

    template <typename PointT, typename DescriptorT>
    pcl::CorrespondencesPtr
    fpcfPointCloudDataPair<PointT, DescriptorT>::getAllPointCorrespondences() const
    {
        return allPointCorrespondences_;
    }

    template <typename PointT, typename DescriptorT>
    pcl::CorrespondencesPtr
    fpcfPointCloudDataPair<PointT, DescriptorT>::getKeypointCorrespondences() const
    {
        return keypointCorrespondences_;
    }

    template <typename PointT, typename DescriptorT>
    Eigen::Matrix4f
    fpcfPointCloudDataPair<PointT, DescriptorT>::getTransformation() const
    {
        return transformation_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudDataPair<PointT, DescriptorT>::setPointCloudSource(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource)
    {
        pointCloudSource_ = pointCloudSource;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudDataPair<PointT, DescriptorT>::setPointCloudTarget(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget)
    {
        pointCloudTarget_ = pointCloudTarget;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudDataPair<PointT, DescriptorT>::setAllPointCorrespondences(pcl::CorrespondencesPtr allPointCorrespondences)
    {
        allPointCorrespondences_ = allPointCorrespondences;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudDataPair<PointT, DescriptorT>::setKeypointCorrespondences(pcl::CorrespondencesPtr keypointCorrespondences)
    {
        keypointCorrespondences_ = keypointCorrespondences;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudDataPair<PointT, DescriptorT>::setTransformation(Eigen::Matrix4f &transformation)
    {
        transformation_ = transformation;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDDATAPAIR_H_