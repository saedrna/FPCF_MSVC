/**
 * @file    fpcfContinuousFusionState.h
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

#ifndef _FPCFCONTINUOUSFUSIONSTATE_H_
#define _FPCFCONTINUOUSFUSIONSTATE_H_

// STL

// Boost

// Qt

// PCL

// FPCF
#include <fpcf_gui/control/guiStates/fpcfState.h>

namespace fpcf_gui
{
    /**
     * This state automatically repeats the fusion process, until the state 
     * machine leaves this state.
     */
    template <typename PointT>
    class fpcfContinuousFusionState : public fpcfState<PointT>
    {
        
        public:
            typedef boost::shared_ptr<fpcfContinuousFusionState<PointT>> Ptr;

            /**
             * Constructor that creates a new continuous fusion state.
             *
             * @param[in] fpcfFusionManager the fusion manager to fuse the point
             *                             clouds
             * @param[in] ui the used fpcf ui 
             * @param[in] vis the visualizer to render the resulting point 
             *                clouds
             */
            fpcfContinuousFusionState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);

            /**
             * Destructor
             */
            virtual ~fpcfContinuousFusionState();

            /**
             * Calls the onUpdate() method
             */
            void onEnter();

            /**
             * Fuses the input point clouds and visualizes the result
             */
            void onUpdate();

            /**
             * Hides the point cloud result from the rendering view
             */
            void onExit();


        protected:

        private:
            // hide default constructor
            fpcfContinuousFusionState();

    };

    template <typename PointT>
    fpcf_gui::fpcfContinuousFusionState<PointT>::fpcfContinuousFusionState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
        : fpcfState(fpcf_gui::GUI_STATE::CONTINUOUS_FUSION, fpcfFusionManager, ui, vis)
    {
    }
            
    template <typename PointT>
    fpcf_gui::fpcfContinuousFusionState<PointT>::~fpcfContinuousFusionState()
    {
        
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfContinuousFusionState<PointT>::onEnter()
    {
        onUpdate();
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfContinuousFusionState<PointT>::onUpdate()
    {
        fpcfFusionManager_->performFusion();
        fpcfPointClouds_ = fpcfFusionManager_->getPointClouds();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            vis_->updatePointCloud(fpcfPointClouds_.at(pcNr)->getPointCloud(), fpcfPointClouds_.at(pcNr)->getId());
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfContinuousFusionState<PointT>::onExit()
    {
        hidePointClouds();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFCONTINUOUSFUSIONSTATE_H_