/**
 * @file    fpcfMainGui.h
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

#ifndef FPCFMAINGUI_H_
#define FPCFMAINGUI_H_

// STL

// Boost

// Qt
#include "ui_fpcfMainGui.h"
#include <QtGui/QMainWindow>
#include <QTimer>

// PCL
#include <pcl/io/pcd_io.h>

// FPCF
#include <fpcf/io/fpcfMultipleKinectGrabber.h>
#include <fpcf/tools/fpcfVisualizer.h>
#include <fpcf_gui/gui/fpcfHelpWindow.h>
#include <fpcf_gui/control/fusion/fpcfFusionManager.h>
#include <fpcf_gui/control/guiStates/fpcfStateMachine.h>

typedef pcl::PointXYZRGBA PointType;

typedef pcl::FPFHSignature33 DescriptorType;

namespace fpcf_gui
{
    /**
     * This class connects the Qt Gui with the fpcf framework.
     */
    class fpcfMainGui : public QMainWindow
    {
        Q_OBJECT

        public:
            /**
             * Constructor that creates a new fpcf gui.
             */
            fpcfMainGui(QWidget *parent = 0, Qt::WFlags flags = 0);

            /**
             * Destructor
             */
            ~fpcfMainGui();

        private:
            Ui::MainGui ui_;
            fpcf_gui::fpcfHelpWindow* helpWindow_;
            fpcf::fpcfVisualizer<PointType, DefaultDescriptorType>::Ptr vis_;
            QTimer *visTimer_;
            fpcf_gui::fpcfFusionManager<PointType>::Ptr fpcfFusionManager_;
            fpcf_gui::fpcfStateMachine<PointType>::Ptr fpcfStateMachine_;
            fpcf::fpcfMultipleKinectGrabber<PointType, DefaultDescriptorType> multiKinectGrabber_;
            std::vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> pointClouds_;
            std::vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> grabbedClouds_;
            std::vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> fusionCloudCopies_;

            void initFusionManager();
            void initTimer();
            void addPointCloud(fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr fpcfPointCloud);
            void removePointCloud(fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr fpcfPointCloud);
            void createPointCloudCopies(std::vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> &pointClouds);

        public slots:
            /**
             * Updates the visualization of the point clouds
             */
            void updatePointClouds();

            /**
             * Changes the state machine to the input state and updates the 
             * visualization
             */
            void toolButton_Input_Clicked();

            /**
             * Changes the state machine to the prepared state and updates the 
             * visualization
             */
            void toolButton_Prepared_Clicked();

            /**
             * Changes the state machine to the single fusion state and updates
             * the visualization
             */
            void toolButton_SingleFusion_Clicked();

            /**
             * Changes the state machine to the continuous fusion state and 
             * updates the visualization
             */
            void toolButton_ContinuousFusion_Clicked();

            /**
             * Changes the state machine to the input state and updates the 
             * visualization
             */
            void toolButton_Mesh_Clicked();

            /**
             * Will be invoked if 'about' from the menu is clicked
             */
            void actionNew_Triggered();

            /**
             * Will be invoked if 'about' from the menu is clicked
             */
            void actionAbout_Triggered();

            /**
             * Will be invoked if 'help' from the menu is clicked
             */
            void actionHelp_Triggered();

            /**
             * Will be invoked if 'quit' from the menu is clicked
             */
            void actionQuit_Triggered();

            /**
             * Will be invoked if 'save as' from the menu is clicked
             */
            void actionSaveAs_Triggered();

            /**
             * Will be invoked, if the persistent features checkbox is clicked
             */
            void checkBox_PersistantFeatures_StateChanged(int i);

            /**
             * Will be invoked, if the point cloud rendering color changes
             */
            void comboBox_Color_CurrentIndexChanged(QString qString);

            /**
             * Will be invoked, if the point descriptor changes
             */
            void comboBox_Descriptor_CurrentIndexChanged(QString qString);

            /**
             * Will be invoked, if the keypoint detector changes
             */
            void comboBox_Detector_CurrentIndexChanged(QString qString);

            /**
             * Will be invoked, if descriptor radius changes
             */
            void doubleSpinBox_DescriptorRadius_ValueChanged(double d);

            /**
             * Will be invoked, if the normal estimation radius changes
             */
            void doubleSpinBox_NormalEstRadius_ValueChanged(double d);

            /**
             * Will be invoked, if the outliner radius for the noise filter 
             * changes
             */
            void doubleSpinBox_OutlinerRadius_ValueChanged(double d);

            /**
             * Will be invoked, if grid size of the voxelgrid filter changes.
             */
            void doubleSpinBox_VoxelgridSize_ValueChanged(double d);

            /**
             * Will be invoked, if the add point cloud button is clicked
             */
            void pushButton_AddPointCloud_Clicked();

            /**
             * Will be invoked, if the capture point cloud from Kinect sensor 
             * is clicked
             */
            void pushButton_CaptureInput_Clicked();

            /**
             * Will be invoked, if start button for the Kinect sensors is 
             * clicked.
             */
            void pushButton_Start_Clicked();

            /**
             * Will be invoked, if stop button for the Kinect sensors is 
             * clicked.
             */
            void pushButton_Stop_Clicked();

            /**
             * Will be invoked, if remove button is clicked
             */
            void pushButton_RemoveAllPointClouds_Clicked();

            /**
             * Will be invoked, if distance of the distance filter changes
             */
            void spinBox_FilterDistance_ValueChanged(int i);

            /**
             * Will be invoked, if the number of iterations of the 
             * LuM-algorithm changes
             */
            void spinBox_GlobalLuMIter_ValueChanged(int i);

            /**
             * Will be invoked, if maximum allowed plane size of a point cloud 
             * changes
             */
            void spinBox_MaxPlaneSize_ValueChanged(int i);

            /**
             * Will be invoked, if the number of iterations of the ICP 
             * algorithm changes
             */
            void spinBox_PairICPIter_ValueChanged(int i);

            /**
             * Will be invoked, if number of iterations of the Ransac 
             * correspondence rejector changes
             */
            void spinBox_RansacIter_ValueChanged(int i);
    };

}

#endif // FPCFMAINGUI_H_
