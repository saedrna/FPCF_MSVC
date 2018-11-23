/**
 * @file    fpcfPairRegistration.h
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

#ifndef _FPCFPAIRREGISTRATION_H_
#define _FPCFPAIRREGISTRATION_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudDataPair.h>
#include <fpcf/filter/fpcfFilter.h>
#include <fpcf/registration/correspondences/fpcfCorrespondenceFinder.h>
#include <fpcf/registration/correspondences/fpcfCorrespondenceRejector.h>
#include <fpcf/registration/fpcfPointCloudPreparation.h>
#include <fpcf/registration/transformation/fpcfICPPairAlignment.h>
#include <fpcf/registration/transformation/fpcfPointCloudPairTransformationEstimation.h>
#include <fpcf/registration/transformation/fpcfPointCloudTransformation.h>

namespace fpcf
{

    /**
     * This class performs all steps to perform a pairwise point cloud 
     * registration. In order to perform all pipeline steps, a point cloud 
     * preparator, a correspondence rejector and a iterative closest point 
     * algorithm object have to be set. Otherwise, the according 
     * steps will be skipped.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPairRegistration
    {
        
        public:

            /**
             * Constructor that creates a new fpcfPairRegistration for a 
             * specified point cloud pair. For a complete pairwise alignment 
             * pipeline, a point cloud preparator, a correspondence rejector 
             * and a iterative closest point algorithm object need to be set 
             * manually.
             *
             * @param[in] fpcfPointCloudPair the used point cloud pair of the 
             *                              pairwise registration
             */
            fpcfPairRegistration(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);

            /**
             * Constructor that creates a new empty fpcfPairRegistration. For a
             * complete pairwise alignment pipeline, a point cloud preparator,
             * a correspondence rejector and a iterative closest point 
             * algorithm object need to be set manually.
             */
            fpcfPairRegistration();

            /**
             * Destructor
             */
            virtual ~fpcfPairRegistration();

            /**
             * Getter for the currently used point cloud pair of the pairwise 
             * registration.
             *
             * @return returns the current point cloud pair
             */
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr getPointCloudPair();

            /**
             * Getter for the resulting registrated point cloud pair after the 
             * performed registration. This is null, if performRegistration() 
             * wasn't invoked beforehand.
             *
             * @return returns the aligned point cloud pair after the 
             *         registration
             */
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr getRegistratedPointCloudPair();

            /**
             * Setter for the used point cloud pair of the pairwise alignment.
             *
             * @param[in] pointCloudPair the used point cloud pair for the 
             *                           pairwise alignment
             */
            void setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Setter for the used point cloud prepartor to prepare the point 
             * cloud. If no point cloud preparator is set, both point clouds 
             * will not be prepared in this class.
             *
             * @param[in] fpcfPointCloudPreparator the point cloud preparator 
             *                                    for the pairwise registration
             */
            void setPointCloudPreparator(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator);

            /**
             * Setter for the used correspondence rejector, which detects and 
             * rejects wrong correspondences of the feature-based alignment. If
             * no correspondence rejector is set, the feature-based alignment 
             * step will be skipped.
             *
             * @param[in] fpcfCorrespondenceRejector the correspondence rejector 
             *                                      to reject wrong 
             *                                      correspondences at the 
             *                                      feature-based alignment
             */
            void setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector);

            /**
             * Setter for the used iterative closest point algorithm, which is 
             * used to fine align both point clouds. If no ICP algorithm object
             * is set, the ICP step will be skipped.
             *
             * @param[in] fpcfIcpPairAlignment the used ICP algorithm object for
             *                                the fine alignment of both point 
             *                                clouds
             */
            void setIcpPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment);

            /**
             * Invokes the registration of the assigned point cloud pair. If no
             * point cloud preperator, a correspondence rejector or ICP 
             * alignment object is set, then the according steps will be 
             * skipped. The resulting point clouds of the pairwise alignment 
             * can be accessed by getRegistratedPointCloudPair(). The computed 
             * transformation of the pairwise alignment can be accessed by 
             * using the getTransformation() method of the point cloud pair.
             */
            void performRegistration();

        protected:
            
        private:
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair_;
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr registratedPcfPointCloudPair_;
            typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator_;
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector_;
            typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment_;

            void setRegistratedPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);

            void preparePointClouds();
            void applyTransformation(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);
            void createRegistrationCopy(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);
            void initialAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);
            void fineAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair);

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::fpcfPairRegistration()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::fpcfPairRegistration(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        fpcfPointCloudPair_ = fpcfPointCloudPair;
    }
      
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::~fpcfPairRegistration()
    {
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::setPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        fpcfPointCloudPair_ = fpcfPointCloudPair;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::setPointCloudPreparator(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator)
    {
        fpcfPointCloudPreparator_ = fpcfPointCloudPreparator;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::setRegistratedPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        registratedPcfPointCloudPair_ = fpcfPointCloudPair;
    }

    
    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector)
    {
        fpcfCorrespondenceRejector_ = fpcfCorrespondenceRejector;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::setIcpPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment)
    {
        fpcfIcpPairAlignment_ = fpcfIcpPairAlignment;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::getPointCloudPair()
    {
        return fpcfPointCloudPair_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::getRegistratedPointCloudPair()
    {
        return registratedPcfPointCloudPair_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::applyTransformation(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        fpcf::fpcfPointCloudTransformation<PointT, DescriptorT> fpcfPointCloudTransformation(fpcfPointCloudPair->getPointCloudSource());
        fpcfPointCloudTransformation.setTransformation(fpcfPointCloudPair->getTransformation());
        fpcfPointCloudTransformation.transformPointCloud();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::performRegistration()
    {
        preparePointClouds();
        createRegistrationCopy(fpcfPointCloudPair_);

        // initial alignment
        Eigen::Matrix4f accumulatedTransformation;

        initialAlignment(registratedPcfPointCloudPair_);
        accumulatedTransformation = registratedPcfPointCloudPair_->getTransformation();

        // fine alignment
        fineAlignment(registratedPcfPointCloudPair_);
        accumulatedTransformation = registratedPcfPointCloudPair_->getTransformation() * accumulatedTransformation;

        // set results
        setRegistratedPointCloudPair(registratedPcfPointCloudPair_);
        fpcfPointCloudPair_->setTransformation(accumulatedTransformation);
        registratedPcfPointCloudPair_->setTransformation(accumulatedTransformation);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::createRegistrationCopy(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSourceCopy(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(*fpcfPointCloudPair->getPointCloudSource()->getMostDownsampledPointCloud()));
        typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTargetCopy(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(*fpcfPointCloudPair->getPointCloudTarget()->getMostDownsampledPointCloud()));
        registratedPcfPointCloudPair_.reset(new fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>(pointCloudSourceCopy, pointCloudTargetCopy));
        registratedPcfPointCloudPair_->setTransformation(fpcfPointCloudPair->getTransformation());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::initialAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        if (fpcfCorrespondenceRejector_)
        {
            fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT> corresTrans(fpcfPointCloudPair);
            Eigen::Matrix4f transformation;
            fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT> corresFind(fpcfPointCloudPair);
            fpcfCorrespondenceRejector_->setPointCloudPair(fpcfPointCloudPair);
            corresFind.calculateCorrepondences();
            fpcfCorrespondenceRejector_->rejectCorrespondences();
            corresTrans.estimateAlignment(transformation);
            fpcfPointCloudPair->setTransformation(transformation);
        } 
        applyTransformation(fpcfPointCloudPair);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::fineAlignment(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair)
    {
        if (fpcfIcpPairAlignment_)
        {
            Eigen::Matrix4f transformation;
            fpcfIcpPairAlignment_->setPointCloudPair(fpcfPointCloudPair);
            fpcfIcpPairAlignment_->calculateTransformation(transformation);
            fpcfPointCloudPair->setTransformation(transformation);
            applyTransformation(fpcfPointCloudPair);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPairRegistration<PointT, DescriptorT>::preparePointClouds()
    {
        if (fpcfPointCloudPreparator_)
        {
            fpcfPointCloudPreparator_->addPointCloud(fpcfPointCloudPair_->getPointCloudSource());
            fpcfPointCloudPreparator_->addPointCloud(fpcfPointCloudPair_->getPointCloudTarget());
            fpcfPointCloudPreparator_->prepare();
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPAIRREGISTRATION_H_