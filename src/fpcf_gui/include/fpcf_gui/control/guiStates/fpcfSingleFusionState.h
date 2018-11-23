/**
 * @file    fpcfSingleFusionState.h
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

#ifndef _FPCFSINGLEFUSIONSTATE_H_
#define _FPCFSINGLEFUSIONSTATE_H_

// STL

// Boost

// Qt

// PCL

// FPCF
#include <fpcf_gui/control/guiStates/fpcfState.h>

namespace fpcf_gui
{
    /**
     * This state performs a single point cloud fusion of the input point 
     * clouds and visualizes the results.
     */
    template <typename PointT>
    class fpcfSingleFusionState : public fpcfState<PointT>
    {
        
        public:
            typedef boost::shared_ptr<fpcfSingleFusionState<PointT>> Ptr;

            /**
             * Constructor that creates a new single fusion state.
             *
             * @param[in] fpcfFusionManager the fusion manager to fuse the point
             *                             clouds
             * @param[in] ui the used fpcf ui 
             * @param[in] vis the visualizer to render the resulting point 
             *                clouds
             */
            fpcfSingleFusionState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);

            /**
             * Destructor
             */
            virtual ~fpcfSingleFusionState();

            /**
             * Calls the onUpdate() method
             */
            void onEnter();

            /**
             * Performs a single fusion of the input point clouds and 
             * visualizes the results.
             */
            void onUpdate();

            /**
             * Removes the fused point clouds from the result.
             */
            void onExit();


        protected:

        private:
            // hide default constructor
            fpcfSingleFusionState();

    };

    template <typename PointT>
    fpcf_gui::fpcfSingleFusionState<PointT>::fpcfSingleFusionState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
        : fpcfState(fpcf_gui::GUI_STATE::SINGLE_FUSION ,fpcfFusionManager, ui, vis)
    {
    }
            
    template <typename PointT>
    fpcf_gui::fpcfSingleFusionState<PointT>::~fpcfSingleFusionState()
    {

    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfSingleFusionState<PointT>::onEnter()
    {
        onUpdate();
    }

    template <typename PointT>
    void 
    fpcfSingleFusionState<PointT>::onUpdate()
    {
        fpcfFusionManager_->performFusion();
        fpcfPointClouds_ = fpcfFusionManager_->getPointClouds();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            if (ui_.checkBox_ShowPrepared->isChecked())
            {
                vis_->updatePointCloud(fpcfPointClouds_.at(pcNr)->getMostDownsampledPointCloud()->getPointCloud(), fpcfPointClouds_.at(pcNr)->getId());
            }
            else
            {
                vis_->updatePointCloud(fpcfPointClouds_.at(pcNr)->getPointCloud(), fpcfPointClouds_.at(pcNr)->getId());
            }
            if (fpcfPointClouds_.at(pcNr)->getMostDownsampledPointCloud()->getKeypointCloud() && ui_.checkBox_ShowKeypoints->isChecked())
            {
                addKeypointCloud(fpcfPointClouds_.at(pcNr));
            }
            else
            {
                removeKeypointCloud(fpcfPointClouds_.at(pcNr));
            }
            if (ui_.checkBox_ShowNormals->isChecked())
            {
                removeNormalCloud(fpcfPointClouds_.at(pcNr));
                addNormalCloud(fpcfPointClouds_.at(pcNr));
            }
            else
            {
                removeNormalCloud(fpcfPointClouds_.at(pcNr));
            }
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfSingleFusionState<PointT>::onExit()
    {
        hidePointClouds();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            removeKeypointCloud(fpcfPointClouds_.at(pcNr));
            removeNormalCloud(fpcfPointClouds_.at(pcNr));
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSINGLEFUSIONSTATE_H_