/**
 * @file    fpcfPreparedState.h
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

#ifndef _FPCFPREPAREDSTATE_H_
#define _FPCFPREPAREDSTATE_H_

// STL

// Boost

// Qt

// PCL

// FPCF
#include <fpcf_gui/control/guiStates/fpcfState.h>

namespace fpcf_gui
{
    /**
     * This state only performs the preparation of the input point clouds and 
     * visualizes the resulting point clouds.
     */
    template <typename PointT>
    class fpcfPreparedState : public fpcfState<PointT>
    {
        
        public:
            typedef boost::shared_ptr<fpcfPreparedState<PointT>> Ptr;

            /**
             * Constructor that creates a new prepared state.
             *
             * @param[in] fpcfFusionManager the fusion manager to fuse the point
             *                             clouds
             * @param[in] ui the used fpcf ui 
             * @param[in] vis the visualizer to render the resulting point 
             *                clouds
             */
            fpcfPreparedState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);
            
            /**
             * Destructor
             */
            virtual ~fpcfPreparedState();

            /**
             * Calls the onUpdate() method
             */
            void onEnter();

            /** 
             * Prepares the input point clouds and visualizes the results.
             */
            void onUpdate();

            /**
             * Hides the prepared point clouds from the visualization.
             */
            void onExit();


        protected:

        private:
            // hide default constructor
            fpcfPreparedState();


    };

    template <typename PointT>
    fpcf_gui::fpcfPreparedState<PointT>::fpcfPreparedState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
        : fpcfState(fpcf_gui::GUI_STATE::PREPARED ,fpcfFusionManager, ui, vis)
    {
    }
            
    template <typename PointT>
    fpcf_gui::fpcfPreparedState<PointT>::~fpcfPreparedState()
    {

    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfPreparedState<PointT>::onEnter()
    {
        onUpdate();
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfPreparedState<PointT>::onUpdate()
    {
        fpcfFusionManager_->prepare();
        fpcfPointClouds_ = fpcfFusionManager_->getPointClouds();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            vis_->updatePointCloud(fpcfPointClouds_.at(pcNr)->getMostDownsampledPointCloud()->getPointCloud(), fpcfPointClouds_.at(pcNr)->getId());
            if (fpcfPointClouds_.at(pcNr)->getMostDownsampledPointCloud()->getKeypointCloud())
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
    fpcf_gui::fpcfPreparedState<PointT>::onExit()
    {
        hidePointClouds();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            removeKeypointCloud(fpcfPointClouds_.at(pcNr));
            removeNormalCloud(fpcfPointClouds_.at(pcNr));
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPREPAREDSTATE_H_