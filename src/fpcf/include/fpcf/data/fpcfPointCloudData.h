/**
 * @file    fpcfPointCloudData.h
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

#ifndef _FPCFPOINTCLOUDDATA_H_
#define _FPCFPOINTCLOUDDATA_H_

// STL
#include <string>

// Boost

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// FPCF

namespace fpcf
{
    /**
     * This class contains all neccessary data of a point cloud for the fusion 
     * process, like downsampled versions of the point cloud, normal 
     * information, keypoints and point descriptors.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudData : public boost::enable_shared_from_this<fpcfPointCloudData<PointT, DescriptorT>>
    {
        
        public:
            typedef boost::shared_ptr<fpcfPointCloudData<PointT, DescriptorT>> Ptr;

            typedef typename pcl::PointCloud<PointT>::Ptr pclPointCloudPtr;
            typedef typename pcl::PointCloud<DescriptorT>::Ptr pclDescriptorCloudPtr;
            typedef pcl::PointCloud<pcl::Normal>::Ptr pclNormalCloudPtr;

            /**
             * Constructor that creates a new empty fpcfPointCloud
             *
             * @param[in] id a unique id for the fpcfPointCloud
             */
            fpcfPointCloudData(std::string id);

            /**
             * Constructor that creates a new fpcfPointCloud from a pclPointCloud
             *
             * @param[in] pclPointCloud the pclPointCloud, which is used to initialize the point cloud data
             * @param[in] id a unique id for the new fpcfPointCloud
             */
            fpcfPointCloudData(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id);

            /**
             * Deep copy constructor
             */
            fpcfPointCloudData(const fpcfPointCloudData& fpcfPointCloudData);

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudData();

            /**
             * Getter for the current pclPointCloud
             *
             * @return returns the current pclPointCloud
             */
            pclPointCloudPtr getPointCloud() const;

            /**
             * Getter for the next downsampled version of the fpcfPointCloud. 
             * This is null, if a fpcfVoxelDownsampling wasn't performed beforehand.
             *
             * @return returns the next downsampled version of the fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getDownsampledPointCloud() const;

            /**
             * Getter for the most downsampled version of the fpcfPointCloud. 
             * This is null, if a fpcfVoxelDownsampling wasn't performed beforehand.
             *
             * @return returns the most downsampled version of the fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getMostDownsampledPointCloud();

            /**
             * Getter for the pclPointCloud, which represent the keypoints. 
             * This is null, if no keypoints were computed before.
             *
             * @return returns the pclPointCloud, which contains the keypoints
             */
            pclPointCloudPtr getKeypointCloud() const;

            /**
             * Getter for the point descriptors of all point of the point cloud
             * This is null, if no point descriptors for all points were 
             * computed before.
             *
             * @return returns the descriptors of all points of the point cloud
             */
            pclDescriptorCloudPtr getPointDescriptorCloud() const;

            /**
             * Getter for the point descriptors of the keypoints of the point cloud
             * This is null, if no point descriptors for the keypoints were 
             * computed before.
             *
             * @return returns the descriptors of keypoints of the point cloud
             */
            pclDescriptorCloudPtr getKeypointDescriptorCloud() const;

            /**
             * Getter for the normals of all points of the point cloud. This is
             * null, if no normals were computed beforehand.
             *
             * @return returns the computed normals of the point cloud
             */
            pclNormalCloudPtr getNormalsCloud() const;

            /**
             * Returns a flag, which indicates that the point cloud already 
             * passed the fpcfPointCloudPreparation class.
             *
             * @return true, if the point cloud is prepared
             */
            bool isPrepared() const;

            /**
             * Getter for the id of the fpcfPointCloud
             *
             * @return return the id of the fpcfPointCloud
             */
            std::string getId() const;

            /**
             * Sets a new pclPointCloud, which represent the actual point cloud
             * in the fpcfPointCloudData. Setting a new pclPointCloud to an old
             * fpcfPointCloud can cause inconsistencies
             *
             * @param[in] pointCloud the new pclPointCloud
             */
            void setPointCloud(pclPointCloudPtr pointCloud);

            /**
             * Sets a new downsampled version of the fpcfPointCloud. This is 
             * used by the fpcfVoxelDownsampling.
             *
             * @param[in] downsampledPointCloud the new downsampled 
             *                                  fpcfPointCloud
             */
            void setDownsampledPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr downsampledPointCloud);

            /**
             * Sets the pclPointCloud, which represent the keypoints of the 
             * keypoints of the point cloud. This is used by the keypoint 
             * detectors.
             *
             * @param[in] keypointCloud the new keypoint cloud
             */
            void setKeypointCloud(pclPointCloudPtr keypointCloud);            

            /**
             * Sets the point descriptors for all points of the point cloud. 
             * This is used by the point descriptors.
             *
             * @param[in] pointDescriptorCloud the new descriptors for all 
             *                                 points of the point cloud
             */
            void setPointDescriptorCloud(pclDescriptorCloudPtr pointDescriptorCloud);            

            /**
             * Sets the point descriptors for the keypoints of the point cloud. 
             * This is used by the point descriptors.
             *
             * @param[in] keypointDescriptorCloud the new descriptors for the
             *                                    keypoints of the point cloud
             */
            void setKeypointDescriptorCloud(pclDescriptorCloudPtr keypointDescriptorCloud);            

            /**
             * Sets the normals for all points of the point cloud. This is used
             * by the fpcfNormalEstimation.
             *
             * @param[in] normalsCloud the new normals for all points of point
             *                         cloud
             */
            void setNormalsCloud(pclNormalCloudPtr normalsCloud);      

            /**
             * Sets a flag, that this fpcfPointCloud is prepared. This is used 
             * by the fpcfPointCloudPreparation.
             *
             * @param[in] prepareState the new preparation state of the point
                                       cloud
             */
            void isPrepared(bool prepareState);

            /**
             * Setter for a unique id of the fpcfPointCloud
             *
             * @param[in] id a new unique id of the point cloud
             */
            void setId(std::string id);

        protected:
            
        private:
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr downsampledPointCloud_;
            pclPointCloudPtr pointCloud_;
            pclPointCloudPtr keypointCloud_;
            pclDescriptorCloudPtr pointDescriptorCloud_;
            pclDescriptorCloudPtr keypointDescriptorCloud_;
            pclNormalCloudPtr normalsCloud_;
            bool isPrepared_;
            std::string id_;

            fpcfPointCloudData(); // hide default constructor
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr makeSharedPtr();
            void init();
    };

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudData<PointT, DescriptorT>::fpcfPointCloudData(std::string id)
    {
        setId(id);
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudData<PointT, DescriptorT>::fpcfPointCloudData(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id)
    {
        setId(id);
        setPointCloud(pclPointCloud);
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudData<PointT, DescriptorT>::fpcfPointCloudData(const fpcfPointCloudData& fpcfPointCloudData)
    {
        if (fpcfPointCloudData.getDownsampledPointCloud())
        {
            fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr downsampledPointCloudCopy(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(*fpcfPointCloudData.getDownsampledPointCloud()));
            setDownsampledPointCloud(downsampledPointCloudCopy);
        }
        if (fpcfPointCloudData.getPointCloud())
        {
            pcl::PointCloud<PointT>::Ptr pointCloudCopy(new pcl::PointCloud<PointT>(*fpcfPointCloudData.getPointCloud()));
            setPointCloud(pointCloudCopy);
        }
        if (fpcfPointCloudData.getKeypointCloud())
        {
            pcl::PointCloud<PointT>::Ptr keypointCloudCopy(new pcl::PointCloud<PointT>(*fpcfPointCloudData.getKeypointCloud()));
            setKeypointCloud(keypointCloudCopy);
        }
        if (fpcfPointCloudData.getPointDescriptorCloud())
        {
            pcl::PointCloud<DescriptorT>::Ptr pointDescriptorCloudCopy(new pcl::PointCloud<DescriptorT>(*fpcfPointCloudData.getPointDescriptorCloud()));
            setPointDescriptorCloud(pointDescriptorCloudCopy);
        }
        if (fpcfPointCloudData.getKeypointDescriptorCloud())
        {
            pcl::PointCloud<DescriptorT>::Ptr keypointDescriptorCloudCopy(new pcl::PointCloud<DescriptorT>(*fpcfPointCloudData.getKeypointDescriptorCloud()));
            setKeypointDescriptorCloud(keypointDescriptorCloudCopy);
        }
        if (fpcfPointCloudData.getNormalsCloud())
        {
            pcl::PointCloud<pcl::Normal>::Ptr normalsCloudCopy(new pcl::PointCloud<pcl::Normal>(*fpcfPointCloudData.getNormalsCloud()));
            setNormalsCloud(normalsCloudCopy);
        }
        isPrepared(fpcfPointCloudData.isPrepared());
        setId(fpcfPointCloudData.getId());
    }
            
    template <typename PointT, typename DescriptorT>
    fpcfPointCloudData<PointT, DescriptorT>::~fpcfPointCloudData()
    {

    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclPointCloudPtr
    fpcfPointCloudData<PointT, DescriptorT>::getPointCloud() const
    {
        return pointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfPointCloudData<PointT, DescriptorT>::getDownsampledPointCloud() const
    {
        return downsampledPointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfPointCloudData<PointT, DescriptorT>::getMostDownsampledPointCloud()
    {
        typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr mostDownsampledPointCloud = makeSharedPtr();
        while(mostDownsampledPointCloud->getDownsampledPointCloud())
        {
            mostDownsampledPointCloud = mostDownsampledPointCloud->getDownsampledPointCloud();
        }
        return mostDownsampledPointCloud;
    }
            
    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclPointCloudPtr
    fpcfPointCloudData<PointT, DescriptorT>::getKeypointCloud() const
    {
        return keypointCloud_;
    }
            
    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclDescriptorCloudPtr
    fpcfPointCloudData<PointT, DescriptorT>::getPointDescriptorCloud() const
    {
        return pointDescriptorCloud_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclDescriptorCloudPtr
    fpcfPointCloudData<PointT, DescriptorT>::getKeypointDescriptorCloud() const
    {
        return keypointDescriptorCloud_;
    }
            
    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclNormalCloudPtr
    fpcfPointCloudData<PointT, DescriptorT>::getNormalsCloud() const
    {
        return normalsCloud_;
    }

    template <typename PointT, typename DescriptorT>
    bool
    fpcfPointCloudData<PointT, DescriptorT>::isPrepared() const
    {
        return isPrepared_;
    }
       
    template <typename PointT, typename DescriptorT>
    std::string
    fpcfPointCloudData<PointT, DescriptorT>::getId() const
    {
        return id_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclPointCloudPtr pointCloud)
    {
        pointCloud_ = pointCloud;
    }
        
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclPointCloudPtr keypointCloud)
    {
        keypointCloud_ = keypointCloud;
    }
            
    template <typename PointT, typename DescriptorT>
    void
        fpcfPointCloudData<PointT, DescriptorT>::setPointDescriptorCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclDescriptorCloudPtr pointDescriptorCloud)
    {
        pointDescriptorCloud_ = pointDescriptorCloud;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setKeypointDescriptorCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclDescriptorCloudPtr keypointDescriptorCloud)
    {
        keypointDescriptorCloud_ = keypointDescriptorCloud;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setDownsampledPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr downsampledPointCloud)
    {
        downsampledPointCloud_ = downsampledPointCloud;
    }
            
    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setNormalsCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::pclNormalCloudPtr normalsCloud)
    {
        normalsCloud_ = normalsCloud;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::isPrepared(bool prepareState)
    {
        isPrepared_ = prepareState;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::setId(std::string id)
    {
        id_ = id;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcfPointCloudData<PointT, DescriptorT>::makeSharedPtr()
    {
        return shared_from_this();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudData<PointT, DescriptorT>::init()
    {
        isPrepared(false);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDDATA_H_