/**
 * @file    fpcfStateMachine.h
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

#ifndef _FPCFSTATEMACHINE_H_
#define _FPCFSTATEMACHINE_H_

// STL

// Boost

// Qt
#include <ui_fpcfMainGui.h>

// PCL
#include <pcl/visualization/pcl_visualizer.h>

// FPCF
#include <fpcf_gui/control/guiStates/fpcfState.h>
#include <fpcf_gui/control/guiStates/fpcfContinuousFusionState.h>
#include <fpcf_gui/control/guiStates/fpcfInputState.h>
#include <fpcf_gui/control/guiStates/fpcfMeshState.h>
#include <fpcf_gui/control/guiStates/fpcfPreparedState.h>
#include <fpcf_gui/control/guiStates/fpcfSingleFusionState.h>

namespace fpcf_gui
{
    /**
     * This class represents a simple state machine. It can change from every 
     * state to any other state. If the state machine changes to a other state,
     * it calls the onExit() method of the old state the onEnter() method of 
     * the new state. If the state machine reenters its current state, it calls
     * the onUpdate() method of this state.
     */
    template <typename PointT>
    class fpcfStateMachine
    {
        public:
            typedef boost::shared_ptr<fpcfStateMachine<PointT>> Ptr;

            /**
             * Constructor that creates a new state machine for the fpcf gui.
             *
             * @param[in] currentStateName the name of the starting state at 
             *                             the instantiation of the state 
             *                             machine
             * @param[in] fpcfFusionManager the fusion manager the states will 
             *                             use to fuse the point clouds
             * @param[in] ui the GUI the states will use
             * @param[in] vis the visualizer the states will use to render 
             *                their results
             */
            fpcfStateMachine(fpcf_gui::GUI_STATE currentStateName, typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);

            /**
             * Destructor
             */
            virtual ~fpcfStateMachine();

            /**
             * Getter for the name of current state of the state machine.
             *
             * @return the name of the current state machine state
             */
            fpcf_gui::GUI_STATE getCurrentStateName();

            /**
             * Getter for the state object, which represents the current state 
             * of the state machine.
             *
             * @return the state object of the current state
             */
            typename fpcf_gui::fpcfState<PointT>::Ptr getCurrentState();

            /**
             * Changes the state of the machine to a new state
             *
             * @param[in] newState the new state of the state machine
             */
            void changeState(fpcf_gui::GUI_STATE newState);

        protected:

        private:
            typename fpcf_gui::fpcfState<PointT>::Ptr currentState_;
            std::map<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr> stateMap_;

            // hide default constructor
            fpcfStateMachine();

    };

    template <typename PointT>
    fpcf_gui::fpcfStateMachine<PointT>::fpcfStateMachine(fpcf_gui::GUI_STATE currentStateName, typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
    {
        stateMap_.insert(std::pair<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr>(fpcf_gui::GUI_STATE::INPUT, new fpcf_gui::fpcfInputState<PointT>(fpcfFusionManager, ui, vis)));
        stateMap_.insert(std::pair<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr>(fpcf_gui::GUI_STATE::PREPARED, new fpcf_gui::fpcfPreparedState<PointT>(fpcfFusionManager, ui, vis)));
        stateMap_.insert(std::pair<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr>(fpcf_gui::GUI_STATE::SINGLE_FUSION, new fpcf_gui::fpcfSingleFusionState<PointT>(fpcfFusionManager, ui, vis)));
        stateMap_.insert(std::pair<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr>(fpcf_gui::GUI_STATE::CONTINUOUS_FUSION, new fpcf_gui::fpcfContinuousFusionState<PointT>(fpcfFusionManager, ui, vis)));
        stateMap_.insert(std::pair<fpcf_gui::GUI_STATE, typename fpcf_gui::fpcfState<PointT>::Ptr>(fpcf_gui::GUI_STATE::MESH, new fpcf_gui::fpcfMeshState<PointT>(fpcfFusionManager, ui, vis)));
        currentState_ = stateMap_.find(currentStateName)->second;
    }

    template <typename PointT>
    fpcf_gui::fpcfStateMachine<PointT>::~fpcfStateMachine()
    {

    }

    template <typename PointT>
    fpcf_gui::GUI_STATE
    fpcf_gui::fpcfStateMachine<PointT>::getCurrentStateName()
    {
        return currentState_->getStateName();
    }

    template <typename PointT>
    typename fpcf_gui::fpcfState<PointT>::Ptr
    fpcf_gui::fpcfStateMachine<PointT>::getCurrentState()
    {
        return currentState_;
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfStateMachine<PointT>::changeState(fpcf_gui::GUI_STATE newState)
    {
        if (currentState_->getStateName() == newState) 
        {
            currentState_->onUpdate();
        }
        else
        {
            currentState_->onExit();
            currentState_ = stateMap_.find(newState)->second;
            currentState_->onEnter();
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSTATEMACHINE_H_