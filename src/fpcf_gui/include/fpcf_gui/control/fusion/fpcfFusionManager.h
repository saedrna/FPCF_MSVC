/**
 * @file    fpcfFusionManager.h
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

#ifndef _FPCFFUSIONMANAGER_H_
#define _FPCFFUSIONMANAGER_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/tools/fpcfPointCloudDataWrapper.h>
#include <fpcf_gui/data/fpcfFusionDataGroup.h>

typedef pcl::FPFHSignature33 DefaultDescriptorType;

namespace fpcf_gui
{
    /**
     * This class ensures that all fusion data groups with different descriptor
     * types contain the same set of parameters. It also selects the right 
     * fusion data group, if the descriptor type changes and starts the fusion
     * process.
     */
    template <typename PointT>
    class fpcfFusionManager
    {
        public:
            typedef boost::shared_ptr<fpcfFusionManager<PointT>> Ptr;

            /**
             * Constructor that creates a new fpcfFusionManager and initializes 
             * all fusion data groups.
             */
            fpcfFusionManager();

            /**
             * Destructor
             */
            virtual ~fpcfFusionManager();
            
            /**
             * Setter for the used point clouds during the fusion process
             * 
             * @param[in] fpcfPointClouds the point clouds for the fusion 
             *                           process
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> &fpcfPointClouds);

            /**
             * Setter for the used size of the voxelgrid filter
             *
             * @param[in] voxelgridSize the new size of the voxelgrid
             */
            void setVoxelgridSize(float voxelgridSize);

            /**
             * Setter for filtering distance of the distance filter
             *
             * @param[in] filterDistance the new filter distance
             */
            void setFilterDistance(float filterDistance);

            /**
             * Setter for the maximum number of points a plane can consist
             *
             * @param[in] maxPlaneSize the maximum number of points a plane is 
             *                         allowed to contain
             */
            void setMaxPlaneSize(int maxPlaneSize);

            /**
             * Setter for the used radius during the outliner collection. If 
             * point has less than two point neighbors in this radius, it will 
             * be removed.
             *
             * @param[in] outlinerRadius the radius of the point neighbor query
             */
            void setOutlinerRadius(float outlinerRadius);

            /**
             * Sets the used radius to collect the point neighbors during the 
             * normal estimation.
             *
             * @param[in] normalEstimationRadius the radius of the radius query
             *                                   during the normal estimation
             */
            void setNormalEstimationRadius(float normalEstimationRadius);

            /**
             * Sets the flag for persistent features computation. If this flag 
             * is set to 'false', the computation of the persistent features 
             * will be skipped.
             *
             * @param[in] calculateState the flag, if the persistent features 
             *                           of the point clouds are computed
             */
            void calculatePersistentFeature(bool calculateState);

            /**
             * Sets the radius, which will be used to describe the points of 
             * the point cloud.
             *
             * @param[in] descriptorRadius the search radius of the descriptor 
             *                             computation
             */
            void setDescriptorRadius(float descriptorRadius);

            /**
             * Can be used to change the keypoint detector during the fusion process
             *
             * @param[in] detectorName "none", "Narf" or "Sift"
             */
            void setDetector(std::string detectorName);

            /**
             * Can be used to change the point descriptor during the fusion process.
             * 
             *
             * @param[in] descriptorName "FPFH", "PFH", "PFHRGB", "Shot" or 
             *                            "Shot Color"
             */
            void setDescriptor(std::string descriptorName);

            /**
             * Setter for the number of Ransac iteration to find the 
             * correspondence inliers.
             *
             * @param[in] ransacIterations the number of Ransac interations
             */
            void setRansacIterations(int ransacIterations);

            /**
             * Sets the maximum number of iterations of the ICP algorithm.
             *
             * @param[in] pairICPIterations the maximum number of ICP 
             *                              iterations
             */
            void setPairICPIterations(int pairICPIterations);

            /**
             * Sets the number of iterations of the global alignment
             *
             * @param[in] globalLuMIterations the number of iterations
             */
            void setGlobalLuMIterations(int globalLuMIterations);

            /**
             * Getter for the current vector of point clouds used in this 
             * fusion manager.
             *
             * @return returns the current point clouds of the fusion process
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> getPointClouds();

            /**
             * Invokes the complete fusion process of the input point clouds.
             */
            void performFusion();

            /**
             * Transforms the input point clouds using the computed 
             * transformations of the last performFusion() call. The ids of the
             * input point clouds needs to be the same as during the fusion 
             * process.
             */
            void transformPointClouds();

            /**
             * Invokes the preparation of the input point clouds.
             */
            void prepare();

            /**
             * Removes the current set keypoint detector. If no keypoint 
             * detector is set, the keypoint detection step will be skipped.
             */
            void removeDetector();

        protected:

        private:
            enum DescriptorType { FPFH, PFH, PFHRGB, Shot, ShotColor };

            std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> *fpcfPointClouds_;

            typename fpcf_gui::fpcfFusionDataGroup<PointT, pcl::FPFHSignature33>::Ptr fpfhFusionDataGroup_;
            typename fpcf_gui::fpcfFusionDataGroup<PointT, pcl::PFHSignature125>::Ptr pfhFusionDataGroup_;
            typename fpcf_gui::fpcfFusionDataGroup<PointT, pcl::PFHRGBSignature250>::Ptr pfhrgbFusionDataGroup_;
            typename fpcf_gui::fpcfFusionDataGroup<PointT, pcl::SHOT352>::Ptr shotFusionDataGroup_;
            typename fpcf_gui::fpcfFusionDataGroup<PointT, pcl::SHOT1344>::Ptr shotColorFusionDataGroup_;
            DescriptorType currentDescriptor_;

            void wrapPointClouds();
            void unwrapPointClouds();
            void init();
            
    };

    template <typename PointT>
    fpcf_gui::fpcfFusionManager<PointT>::fpcfFusionManager()
    {
        init();
    }

    template <typename PointT>
    fpcf_gui::fpcfFusionManager<PointT>::~fpcfFusionManager()
    {
        
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> &fpcfPointClouds)
    {
        fpcfPointClouds_ = &fpcfPointClouds;
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfFusionManager<PointT>::setVoxelgridSize(float voxelgridSize)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setVoxelgridSize(voxelgridSize);
        pfhFusionDataGroup_->getGuiPreparator()->setVoxelgridSize(voxelgridSize);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setVoxelgridSize(voxelgridSize);
        shotFusionDataGroup_->getGuiPreparator()->setVoxelgridSize(voxelgridSize);
        shotColorFusionDataGroup_->getGuiPreparator()->setVoxelgridSize(voxelgridSize);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setFilterDistance(float filterDistance)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setFilterDistance(filterDistance);
        pfhFusionDataGroup_->getGuiPreparator()->setFilterDistance(filterDistance);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setFilterDistance(filterDistance);
        shotFusionDataGroup_->getGuiPreparator()->setFilterDistance(filterDistance);
        shotColorFusionDataGroup_->getGuiPreparator()->setFilterDistance(filterDistance);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setMaxPlaneSize(int maxPlaneSize)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setMaxPlaneSize(maxPlaneSize);
        pfhFusionDataGroup_->getGuiPreparator()->setMaxPlaneSize(maxPlaneSize);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setMaxPlaneSize(maxPlaneSize);
        shotFusionDataGroup_->getGuiPreparator()->setMaxPlaneSize(maxPlaneSize);
        shotColorFusionDataGroup_->getGuiPreparator()->setMaxPlaneSize(maxPlaneSize);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setOutlinerRadius(float outlinerRadius)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setOutlinerRadius(outlinerRadius);
        pfhFusionDataGroup_->getGuiPreparator()->setOutlinerRadius(outlinerRadius);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setOutlinerRadius(outlinerRadius);
        shotFusionDataGroup_->getGuiPreparator()->setOutlinerRadius(outlinerRadius);
        shotColorFusionDataGroup_->getGuiPreparator()->setOutlinerRadius(outlinerRadius);
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfFusionManager<PointT>::setNormalEstimationRadius(float normalEstimationRadius)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setNormalEstimationRadius(normalEstimationRadius);
        pfhFusionDataGroup_->getGuiPreparator()->setNormalEstimationRadius(normalEstimationRadius);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setNormalEstimationRadius(normalEstimationRadius);
        shotFusionDataGroup_->getGuiPreparator()->setNormalEstimationRadius(normalEstimationRadius);
        shotColorFusionDataGroup_->getGuiPreparator()->setNormalEstimationRadius(normalEstimationRadius);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::calculatePersistentFeature(bool calculateState)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->calculatePersistentFeature(calculateState);
        pfhFusionDataGroup_->getGuiPreparator()->calculatePersistentFeature(calculateState);
        pfhrgbFusionDataGroup_->getGuiPreparator()->calculatePersistentFeature(calculateState);
        shotFusionDataGroup_->getGuiPreparator()->calculatePersistentFeature(calculateState);
        shotColorFusionDataGroup_->getGuiPreparator()->calculatePersistentFeature(calculateState);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setDescriptorRadius(float descriptorRadius)
    {
        fpfhFusionDataGroup_->getGuiPreparator()->setDescriptorRadius(descriptorRadius);
        pfhFusionDataGroup_->getGuiPreparator()->setDescriptorRadius(descriptorRadius);
        pfhrgbFusionDataGroup_->getGuiPreparator()->setDescriptorRadius(descriptorRadius);
        shotFusionDataGroup_->getGuiPreparator()->setDescriptorRadius(descriptorRadius);
        shotColorFusionDataGroup_->getGuiPreparator()->setDescriptorRadius(descriptorRadius);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setDetector(std::string detectorName)
    {
        fpfhFusionDataGroup_->setDetector(detectorName);
        pfhFusionDataGroup_->setDetector(detectorName);
        pfhrgbFusionDataGroup_->setDetector(detectorName);
        shotFusionDataGroup_->setDetector(detectorName);
        shotColorFusionDataGroup_->setDetector(detectorName);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setDescriptor(std::string descriptorName)
    {
        if (descriptorName == "FPFH")
        {
            currentDescriptor_ = DescriptorType::FPFH;
        }
        else if (descriptorName == "PFH")
        {
            currentDescriptor_ = DescriptorType::PFH;
        }
        else if (descriptorName == "PFHRGB")
        {
            currentDescriptor_ = DescriptorType::PFHRGB;
        }
        else if (descriptorName == "Shot")
        {
            currentDescriptor_ = DescriptorType::Shot;
        }
        else if (descriptorName == "Shot Color")
        {
            currentDescriptor_ = DescriptorType::ShotColor;
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setRansacIterations(int ransacIterations)
    {
        fpfhFusionDataGroup_->getCorrespondenceRejector()->setRansacIterations(ransacIterations);
        pfhFusionDataGroup_->getCorrespondenceRejector()->setRansacIterations(ransacIterations);
        pfhrgbFusionDataGroup_->getCorrespondenceRejector()->setRansacIterations(ransacIterations);
        shotFusionDataGroup_->getCorrespondenceRejector()->setRansacIterations(ransacIterations);
        shotColorFusionDataGroup_->getCorrespondenceRejector()->setRansacIterations(ransacIterations);
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfFusionManager<PointT>::setPairICPIterations(int pairICPIterations)
    {
        fpfhFusionDataGroup_->getIcpPairAlignment()->setIterations(pairICPIterations);
        pfhFusionDataGroup_->getIcpPairAlignment()->setIterations(pairICPIterations);
        pfhrgbFusionDataGroup_->getIcpPairAlignment()->setIterations(pairICPIterations);
        shotFusionDataGroup_->getIcpPairAlignment()->setIterations(pairICPIterations);
        shotColorFusionDataGroup_->getIcpPairAlignment()->setIterations(pairICPIterations);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::setGlobalLuMIterations(int globalLuMIterations)
    {
        fpfhFusionDataGroup_->getLuMGlobalAlignment()->setIterations(globalLuMIterations);
        pfhFusionDataGroup_->getLuMGlobalAlignment()->setIterations(globalLuMIterations);
        pfhrgbFusionDataGroup_->getLuMGlobalAlignment()->setIterations(globalLuMIterations);
        shotFusionDataGroup_->getLuMGlobalAlignment()->setIterations(globalLuMIterations);
        shotColorFusionDataGroup_->getLuMGlobalAlignment()->setIterations(globalLuMIterations);
    }

    template <typename PointT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> 
    fpcf_gui::fpcfFusionManager<PointT>::getPointClouds()
    {
        return *fpcfPointClouds_;
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::performFusion()
    {
        wrapPointClouds();
        if (currentDescriptor_ == DescriptorType::FPFH)
        {
            std::cout << "Perform FPFH registration" << std::endl;
            fpfhFusionDataGroup_->performFusion();
        }
        else if (currentDescriptor_ == DescriptorType::PFH)
        {
            std::cout << "Perform PFH registration" << std::endl;
            pfhFusionDataGroup_->performFusion();
        }
        else if (currentDescriptor_ == DescriptorType::PFHRGB)
        {
            std::cout << "Perform PFHRGB registration" << std::endl;
            pfhrgbFusionDataGroup_->performFusion();
        }
        else if (currentDescriptor_ == DescriptorType::Shot)
        {
            std::cout << "Perform Shot registration" << std::endl;
            shotFusionDataGroup_->performFusion();
        }
        else if (currentDescriptor_ == DescriptorType::ShotColor)
        {
            std::cout << "Perform Shot Color registration" << std::endl;
            shotColorFusionDataGroup_->performFusion();
        }
        unwrapPointClouds();
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::transformPointClouds()
    {
        wrapPointClouds();
        if (currentDescriptor_ == DescriptorType::FPFH)
        {
            fpfhFusionDataGroup_->transformPointCloud();
        }
        else if (currentDescriptor_ == DescriptorType::PFH)
        {
            pfhFusionDataGroup_->transformPointCloud();
        }
        else if (currentDescriptor_ == DescriptorType::PFHRGB)
        {
            pfhrgbFusionDataGroup_->transformPointCloud();
        }
        else if (currentDescriptor_ == DescriptorType::Shot)
        {
            shotFusionDataGroup_->transformPointCloud();
        }
        else if (currentDescriptor_ == DescriptorType::ShotColor)
        {
            shotColorFusionDataGroup_->transformPointCloud();
        }
        unwrapPointClouds();
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::prepare()
    {
        wrapPointClouds();
        if (currentDescriptor_ == DescriptorType::FPFH)
        {
            std::cout << "Perform FPFH prepare" << std::endl;
            fpfhFusionDataGroup_->prepare();
        }
        else if (currentDescriptor_ == DescriptorType::PFH)
        {
            std::cout << "Perform PFH prepare" << std::endl;
            pfhFusionDataGroup_->prepare();
        }
        else if (currentDescriptor_ == DescriptorType::PFHRGB)
        {
            std::cout << "Perform PFHRGB prepare" << std::endl;
            pfhrgbFusionDataGroup_->prepare();
        }
        else if (currentDescriptor_ == DescriptorType::Shot)
        {
            std::cout << "Perform Shot prepare" << std::endl;
            shotFusionDataGroup_->prepare();
        }
        else if (currentDescriptor_ == DescriptorType::ShotColor)
        {
            std::cout << "Perform Shot Color prepare" << std::endl;
            shotColorFusionDataGroup_->prepare();
        }
        unwrapPointClouds();
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfFusionManager<PointT>::wrapPointClouds()
    {
        if (currentDescriptor_ == DescriptorType::FPFH)
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, pcl::FPFHSignature33>::Ptr> wrappedPointCloudsFpfh;
            fpcf::wrapPointCloudsData<PointT, DefaultDescriptorType, pcl::FPFHSignature33>(*fpcfPointClouds_, wrappedPointCloudsFpfh);
            fpfhFusionDataGroup_->setPointClouds(wrappedPointCloudsFpfh);
        }
        else if (currentDescriptor_ == DescriptorType::PFH)
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, pcl::PFHSignature125>::Ptr> wrappedPointCloudsPfh;
            fpcf::wrapPointCloudsData<PointT, DefaultDescriptorType, pcl::PFHSignature125>(*fpcfPointClouds_, wrappedPointCloudsPfh);
            pfhFusionDataGroup_->setPointClouds(wrappedPointCloudsPfh);
        }
        else if (currentDescriptor_ == DescriptorType::PFHRGB)
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, pcl::PFHRGBSignature250>::Ptr> wrappedPointCloudsPfhrgb;
            fpcf::wrapPointCloudsData<PointT, DefaultDescriptorType, pcl::PFHRGBSignature250>(*fpcfPointClouds_, wrappedPointCloudsPfhrgb);
            pfhrgbFusionDataGroup_->setPointClouds(wrappedPointCloudsPfhrgb);
        }
        else if (currentDescriptor_ == DescriptorType::Shot)
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, pcl::SHOT352>::Ptr> wrappedPointCloudsShot;
            fpcf::wrapPointCloudsData<PointT, DefaultDescriptorType, pcl::SHOT352>(*fpcfPointClouds_, wrappedPointCloudsShot);
            shotFusionDataGroup_->setPointClouds(wrappedPointCloudsShot);
        }
        else if (currentDescriptor_ == DescriptorType::ShotColor)
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, pcl::SHOT1344>::Ptr> wrappedPointCloudsShot;
            fpcf::wrapPointCloudsData<PointT, DefaultDescriptorType, pcl::SHOT1344>(*fpcfPointClouds_, wrappedPointCloudsShot);
            shotColorFusionDataGroup_->setPointClouds(wrappedPointCloudsShot);
        }
    }

    
    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::unwrapPointClouds()
    {
        if (currentDescriptor_ == DescriptorType::FPFH)
        {
            fpcf::wrapPointCloudsData<PointT, pcl::FPFHSignature33, DefaultDescriptorType>(fpfhFusionDataGroup_->getPointClouds(), *fpcfPointClouds_);
        }
        else if (currentDescriptor_ == DescriptorType::PFH)
        {
            fpcf::wrapPointCloudsData<PointT, pcl::PFHSignature125, DefaultDescriptorType>(pfhFusionDataGroup_->getPointClouds(), *fpcfPointClouds_);
        }
        else if (currentDescriptor_ == DescriptorType::PFHRGB)
        {
            fpcf::wrapPointCloudsData<PointT, pcl::PFHRGBSignature250, DefaultDescriptorType>(pfhrgbFusionDataGroup_->getPointClouds(), *fpcfPointClouds_);
        }
        else if (currentDescriptor_ == DescriptorType::Shot)
        {
            fpcf::wrapPointCloudsData<PointT, pcl::SHOT352, DefaultDescriptorType>(shotFusionDataGroup_->getPointClouds(), *fpcfPointClouds_);
        }
        else if (currentDescriptor_ == DescriptorType::ShotColor)
        {
            fpcf::wrapPointCloudsData<PointT, pcl::SHOT1344, DefaultDescriptorType>(shotColorFusionDataGroup_->getPointClouds(), *fpcfPointClouds_);
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfFusionManager<PointT>::init()
    {
        const float DEFAULT_DESCRIPTOR_RADIUS = 0.035f;

        fpfhFusionDataGroup_.reset(new fpcf_gui::fpcfFusionDataGroup<PointT, pcl::FPFHSignature33>());
        pfhFusionDataGroup_.reset(new fpcf_gui::fpcfFusionDataGroup<PointT, pcl::PFHSignature125>());
        pfhrgbFusionDataGroup_.reset(new fpcf_gui::fpcfFusionDataGroup<PointT, pcl::PFHRGBSignature250>());
        shotFusionDataGroup_.reset(new fpcf_gui::fpcfFusionDataGroup<PointT, pcl::SHOT352>());
        shotColorFusionDataGroup_.reset(new fpcf_gui::fpcfFusionDataGroup<PointT, pcl::SHOT1344>());
        currentDescriptor_ = DescriptorType::FPFH;
        typename fpcf::fpcfFPFHDescriptor<PointT>::Ptr fpfhDescriptor(new fpcf::fpcfFPFHDescriptor<PointT>(DEFAULT_DESCRIPTOR_RADIUS));
        typename fpcf::fpcfPFHDescriptor<PointT>::Ptr pfhDescriptor(new fpcf::fpcfPFHDescriptor<PointT>(DEFAULT_DESCRIPTOR_RADIUS));
        typename fpcf::fpcfPFHRGBDescriptor<PointT>::Ptr pfhrgbDescriptor(new fpcf::fpcfPFHRGBDescriptor<PointT>(DEFAULT_DESCRIPTOR_RADIUS));
        typename fpcf::fpcfShotDescriptor<PointT>::Ptr shotDescriptor(new fpcf::fpcfShotDescriptor<PointT>(DEFAULT_DESCRIPTOR_RADIUS));
        typename fpcf::fpcfShotColorDescriptor<PointT>::Ptr shotColorDescriptor(new fpcf::fpcfShotColorDescriptor<PointT>(DEFAULT_DESCRIPTOR_RADIUS));
        fpfhFusionDataGroup_->setDescriptor(fpfhDescriptor);
        pfhFusionDataGroup_->setDescriptor(pfhDescriptor);
        pfhrgbFusionDataGroup_->setDescriptor(pfhrgbDescriptor);
        shotFusionDataGroup_->setDescriptor(shotDescriptor);
        shotColorFusionDataGroup_->setDescriptor(shotColorDescriptor);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFFUSIONMANAGER_H_