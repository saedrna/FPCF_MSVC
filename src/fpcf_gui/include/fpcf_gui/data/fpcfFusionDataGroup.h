/**
 * @file    fpcfFusionDataGroup.h
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

#ifndef _FPCFFUSIONDATAGROUP_H_
#define _FPCFFUSIONDATAGROUP_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/registration/features/descriptors/fpcfFPFHDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfPFHDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfPFHRGBDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfShotDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfShotColorDescriptor.h>
#include <fpcf/registration/features/detectors/fpcfNarfKeypoint.h>
#include <fpcf/registration/features/detectors/fpcfSiftKeypoint.h>
#include <fpcf/registration/correspondences/fpcfCorrespondenceRejector.h>
#include <fpcf/registration/transformation/fpcfICPPairAlignment.h>
#include <fpcf/registration/transformation/fpcfLuMGlobalAlignment.h>
#include <fpcf/registration/fpcfMultipleRegistration.h>
#include <fpcf_gui/control/fusion/fpcfGuiMultipleFusion.h>
#include <fpcf_gui/control/fusion/fpcfGuiPointCloudPreparation.h>

namespace fpcf_gui
{
    /**
     * This class contains the parameters and algorithm to fuse a point cloud 
     * with a specified point type and descriptor type.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfFusionDataGroup
    {
        public:
            typedef boost::shared_ptr<fpcfFusionDataGroup<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new empty fusion data group
             */
            fpcfFusionDataGroup();

            /**
             * Destructor
             */
            virtual ~fpcfFusionDataGroup();

            /**
             * Setter for the used multiple fusion algorithm to fuse the input 
             * point clouds.
             *
             * @param[in] fpcfGuiMultipleFusion the multiple fusion object
             */
            void setMultipleFusion(typename fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::Ptr fpcfGuiMultipleFusion);

            /**
             * Sets the used point cloud preparator during the multiple 
             * registration of the point clouds.
             *
             * @param[in] fpcfGuiPreparator the preparator to prepare the input 
             *                             point clouds
             */
            void setGuiPreparator(typename fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfGuiPreparator);

            /**
             * Sets the used correspondence rejector during the multiple 
             * registration of the point clouds.
             *
             * @param[in] fpcfCorrespondenceRejector the preparator to prepare 
             *                                      the input point clouds
             */
            void setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector);

            /**
             * Sets the used iterative closest point algorithm during the 
             * multiple registration of the point clouds.
             *
             * @param[in] fpcfIcpPairAlignment the ICP algorithm for the fine 
             *                                alignment of two point clouds
             */
            void setIcpPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment);

            /**
             * Sets the used global alignment algorithm during the multiple 
             * registration of the point clouds.
             *
             * @param[in] fpcfLuMGlobalAlignment the global alignment algorithm 
             *                                  to reduce the accumulated error
             *                                  of the successive pairwise 
             *                                  alignment
             */
            void setLuMGlobalAlignment(typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment);

            /**
             * Can be used to change the keypoint detector during the fusion process
             *
             * @param[in] detectorName "none", "Narf" or "Sift"
             */
            void setDetector(std::string detectorName);

            /**
             * Setter for a point descriptor algorithm, which will be used to 
             * compute the point descriptors of the point clouds.
             *
             * @param[in] fpcfDescriptor the point descriptor algorithm to 
             *                          describe the points of the point clouds
             */
            void setDescriptor(typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor);

            /**
             * Sets the vector of point clouds for the preparation process.
             *
             * @param[in] pointClouds the vector of point clouds, which will be
             *                        prepared
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds);
            
            /**
             * Getter for the current multiple fusion algorithm to fuse the 
             * input point clouds.
             *
             * @return returns the current multiple fusion object
             */
            typename fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::Ptr getMultipleFusion();

            /**
             * Getter for the currently used point cloud preparator to prepare
             * the input point cloud.
             *
             * @return returns the current point cloud preparator
             */
            typename fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::Ptr getGuiPreparator();

            /**
             * Getter for the currently used correspondence rejector to reject 
             * wrong corresponding point pairs the input point cloud.
             *
             * @return returns the current correspondence rejector
             */
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr getCorrespondenceRejector();

            /**
             * Getter for the currently used iteratice closest point algorithm
             * to fine align the input point clouds.
             *
             * @return returns the current ICP algorithm object
             */
            typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr getIcpPairAlignment();

            /**
             * Getter for the currently used global alignment algorithm to 
             * reduce the accumulated error of the successive pairwise 
             * alignment
             *
             * @return returns the current ICP algorithm object
             */
            typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr getLuMGlobalAlignment();

            /**
             * Getter for the currently used fpcfPointClouds of the multiple 
             * registration
             *
             * @return returns the vector of pointClouds
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> getPointClouds();

            /**
             * Invokes the complete fusion process of the input point clouds.
             */
            void performFusion();

            /**
             * Invokes the preparation of the input point clouds.
             */
            void prepare();

            /**
             * Transforms the input point clouds using the computed 
             * transformations of the last performFusion() call. The ids of the
             * input point clouds needs to be the same as during the fusion 
             * process.
             */
            void transformPointCloud();

        protected:

        private:
            typename fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::Ptr fpcfGuiMultipleFusion_;
            typename fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfGuiPreparator_;
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector_;
            typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment_;
            typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment_;
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds_;

            void init();
    };

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::fpcfFusionDataGroup()
    {
        init();
    }

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::~fpcfFusionDataGroup()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setMultipleFusion(typename fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::Ptr fpcfGuiMultipleFusion)
    {
        fpcfGuiMultipleFusion_ = fpcfGuiMultipleFusion;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setGuiPreparator(typename fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfGuiPreparator)
    {
        fpcfGuiPreparator_ = fpcfGuiPreparator;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector)
    {
        fpcfCorrespondenceRejector_ = fpcfCorrespondenceRejector;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setIcpPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment)
    {
        fpcfIcpPairAlignment_ = fpcfIcpPairAlignment;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setLuMGlobalAlignment(typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment)
    {
        fpcfLuMGlobalAlignment_ = fpcfLuMGlobalAlignment;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds)
    {
        pointClouds_ = pointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setDetector(std::string detectorName)
    {
        if (detectorName == "none")
        {
            fpcfGuiPreparator_->removeDetector();
        }
        else if (detectorName == "Narf")
        {
            fpcf::fpcfNarfKeypoint<PointT, DescriptorT>::Ptr fpcfNarfKeypoint(new fpcf::fpcfNarfKeypoint<PointT, DescriptorT>());
            fpcfGuiPreparator_->setDetector(fpcfNarfKeypoint);
        }
        else if (detectorName == "Sift")
        {
            fpcf::fpcfSiftKeypoint<PointT, DescriptorT>::Ptr fpcfSiftKeypoint(new fpcf::fpcfSiftKeypoint<PointT, DescriptorT>());
            fpcfGuiPreparator_->setDetector(fpcfSiftKeypoint);
        }
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::setDescriptor(typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor)
    {
        fpcfGuiPreparator_->setDescriptor(fpcfDescriptor);
    }
            
    template <typename PointT, typename DescriptorT>
    typename fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::Ptr 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getMultipleFusion()
    {
        return fpcfGuiMultipleFusion_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>::Ptr 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getGuiPreparator()
    {
        return fpcfGuiPreparator_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getCorrespondenceRejector()
    {
        return fpcfCorrespondenceRejector_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getIcpPairAlignment()
    {
        return fpcfIcpPairAlignment_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getLuMGlobalAlignment()
    {
        return fpcfLuMGlobalAlignment_;
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::getPointClouds()
    {
        return pointClouds_;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::performFusion()
    {
        fpcfGuiMultipleFusion_->setPointClouds(pointClouds_);
        fpcfGuiMultipleFusion_->performRegistration();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::prepare()
    {
        fpcfGuiPreparator_->setPointClouds(pointClouds_);
        fpcfGuiPreparator_->prepare();
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::transformPointCloud()
    {
        fpcfGuiMultipleFusion_->setPointClouds(pointClouds_);
        fpcfGuiMultipleFusion_->transformPointClouds();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfFusionDataGroup<PointT, DescriptorT>::init()
    {
        fpcfGuiPreparator_.reset(new fpcf_gui::fpcfGuiPointCloudPreparation<PointT, DescriptorT>());
        fpcfCorrespondenceRejector_.reset(new fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>());
        fpcfLuMGlobalAlignment_.reset(new fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>());
        fpcfIcpPairAlignment_.reset(new fpcf::fpcfICPPairAlignment<PointT, DescriptorT>());
        fpcfGuiMultipleFusion_.reset(new fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>(fpcfGuiPreparator_));
        fpcfGuiMultipleFusion_->setCorrespondenceRejector(fpcfCorrespondenceRejector_);
        fpcfGuiMultipleFusion_->setICPPairAlignment(fpcfIcpPairAlignment_);
        fpcfGuiMultipleFusion_->setLuMGlobalAlignment(fpcfLuMGlobalAlignment_);   
    }

} // end namespace fpcf

#endif // #ifndef _FPCFFUSIONDATAGROUP_H_