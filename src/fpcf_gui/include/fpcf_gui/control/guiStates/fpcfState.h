/**
 * @file    fpcfState.h
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

#ifndef _FPCFSTATE_H_
#define _FPCFSTATE_H_

// STL

// Boost
#include <boost/shared_ptr.hpp>

// Qt
#include <ui_fpcfMainGui.h>

// PCL

// FPCF
#include <fpcf/tools/fpcfVisualizer.h>
#include <fpcf_gui/control/fusion/fpcfFusionManager.h>

namespace fpcf_gui
{
    /**
     * Enumeration, which represents the different states of the GUI state 
     * machine.
     */
    enum GUI_STATE { INPUT, PREPARED, SINGLE_FUSION, CONTINUOUS_FUSION, MESH };

    /**
     * This is an abstract class, which represents a state of the GUI state 
     * machine. A state of the state machine can perform different actions on 
     * enter, on update and on exit.
     */
    template <typename PointT>
    class fpcfState
    {
        
        public:
            typedef boost::shared_ptr<fpcfState<PointT>> Ptr;

            /**
             * Destructor
             */
            virtual ~fpcfState();

            /**
             * This method will be automatically invoked, if the state machine 
             * enters this state.
             */
            virtual void onEnter() = 0;

            /**
             * This method will be automatically invoked, if the state machine 
             * reenters this state.
             */
            virtual void onUpdate() = 0;

            /**
             * This method will be automatically invoked, if the state machine 
             * exits this state.
             */
            virtual void onExit() = 0;

            /**
             * Setter for the used fusion manager in this state to perform 
             * fusion methods.
             *
             * @param[in] fpcfFusionManager the fusion manager, which will be 
             *                             used
             */
            void setFusionManager(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager);

            /**
             * Setter for the used visualizer to change the rendering of 
             * results.
             *
             * @param[in] vis the visualizer, which will be used
             */
            void setVisualizer(typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);

            /**
             * Setter for the used GUI of the point cloud fusion framework.
             *
             * @param[in] ui the fpcf GUI
             */
            void setUi(Ui::MainGui ui);

            /**
             * Getter for the current fusion manager in this state
             *
             * @return returns the current fusion manager
             */
            typename fpcf_gui::fpcfFusionManager<PointT>::Ptr getFusionManager();

            /**
             * Getter for the current visualizer in this state
             *
             * @return returns the current visualizer
             */
            typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr getVisualizer();

            /**
             * Getter for the fpcf GUI in this state
             *
             * @return returns the fpcf GUI
             */
            typename Ui::MainGui getUi();

            /**
             * Getter for the name of the state, which is represented.
             *
             * @return returns the name of the represented state
             */
            fpcf_gui::GUI_STATE getStateName();

        protected:
            typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager_;
            Ui::MainGui ui_;
            typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis_;
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr> fpcfPointClouds_;

            fpcfState(fpcf_gui::GUI_STATE stateName, typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);

            void hidePointClouds();
            void addKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud);
            void addNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud);
            void removeKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud);
            void removeNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud);

        private:
            fpcf_gui::GUI_STATE stateName_;

            // hide default constructor
            fpcfState();

    };

    template <typename PointT>
    fpcf_gui::fpcfState<PointT>::fpcfState(fpcf_gui::GUI_STATE stateName, typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
    {
        stateName_ = stateName;
        setFusionManager(fpcfFusionManager);
        setUi(ui);
        setVisualizer(vis);
    }
            
    template <typename PointT>
    fpcf_gui::fpcfState<PointT>::~fpcfState()
    {

    }

    template <typename PointT>
    void
    fpcf_gui::fpcfState<PointT>::setFusionManager(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager)
    {
        fpcfFusionManager_ = fpcfFusionManager;
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfState<PointT>::setVisualizer(typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
    {
        vis_ = vis;
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfState<PointT>::setUi(Ui::MainGui ui)
    {
        ui_ = ui;
    }

    template <typename PointT>
    typename fpcf_gui::fpcfFusionManager<PointT>::Ptr
    fpcf_gui::fpcfState<PointT>::getFusionManager()
    {
        fpcfFusionManager_ = fpcfFusionManager;
    }

    template <typename PointT>
    typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr
    fpcf_gui::fpcfState<PointT>::getVisualizer()
    {
        return vis_;
    }

    template <typename PointT>
    Ui::MainGui
    fpcf_gui::fpcfState<PointT>::getUi()
    {
        return ui_;
    }

    template <typename PointT>
    fpcf_gui::GUI_STATE 
    fpcf_gui::fpcfState<PointT>::getStateName()
    {
        return stateName_;
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfState<PointT>::hidePointClouds()
    {
        pcl::PointCloud<PointT>::Ptr emptyPointCloud(new pcl::PointCloud<PointT>());
        emptyPointCloud->clear();
        for (int pcNr = 0; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
             vis_->updatePointCloud(emptyPointCloud, fpcfPointClouds_.at(pcNr)->getId());
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfState<PointT>::addKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud)
    {
        const double POINT_SIZE = 7;
        double r = rand() % 255;
        double g = rand() % 255;
        double b = rand() % 255;
        std::string id = fpcfPointCloud->getId() + "keypoints";
        pcl::visualization::PointCloudColorHandlerCustom<PointType>::Ptr pointCloudColorHandler(new pcl::visualization::PointCloudColorHandlerCustom<PointType>(fpcfPointCloud->getMostDownsampledPointCloud()->getKeypointCloud(), r, g, b));
        ((pcl::visualization::PCLVisualizer)*vis_).updatePointCloud(fpcfPointCloud->getMostDownsampledPointCloud()->getKeypointCloud(), *pointCloudColorHandler, id);
        vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, id);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfState<PointT>::addNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud)
    {
        vis_->addPointCloudNormals<PointType, pcl::Normal>(fpcfPointCloud->getMostDownsampledPointCloud()->getPointCloud(), fpcfPointCloud->getMostDownsampledPointCloud()->getNormalsCloud(), 10, 0.05f, fpcfPointCloud->getId() + "normals");
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfState<PointT>::removeKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud)
    {
        pcl::PointCloud<PointType>::Ptr keypointCloudPlaceholder(new pcl::PointCloud<PointType>());
        vis_->updatePointCloud(keypointCloudPlaceholder, fpcfPointCloud->getId() + "keypoints");
    }

    template <typename PointT>
    void
    fpcf_gui::fpcfState<PointT>::removeNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloud)
    {
        vis_->removePointCloud(fpcfPointCloud->getId() + "normals");
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSTATE_H_