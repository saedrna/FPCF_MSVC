/**
 * @file    fpcfMainGui.cpp
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
#include "fpcf_gui/fpcfGuiPrecompiled.h" // precompiled header
#include "fpcf_gui/gui/fpcfMainGui.h"

// STL

// Boost

// Qt
#include <QFileDialog>
#include <QMessageBox>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>

// PCL
#include <pcl/io/ply_io.h>

// fpcf
#include <fpcf/io/fpcfPointCloudReader.h>
#include <fpcf/tools/fpcfPointCloudConcatenator.h>
#include <fpcf/tools/fpcfPointCloudDataWrapper.h>

fpcf_gui::fpcfMainGui::fpcfMainGui(QWidget *parent, Qt::WFlags flags)
    : QMainWindow(parent, flags)
{
    ui_.setupUi(this);
    ui_.label_KinectSensorInitialization->hide();
    vis_.reset(new fpcf::fpcfVisualizer<PointType, DefaultDescriptorType>(false));
    ui_.widget_Visualizer->SetRenderWindow(vis_->getRenderWindow().GetPointer());
    vis_->setupInteractor(ui_.widget_Visualizer->GetInteractor(), ui_.widget_Visualizer->GetRenderWindow());
    vis_->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    initFusionManager();
    fpcfStateMachine_.reset(new fpcf_gui::fpcfStateMachine<PointType>(fpcf_gui::GUI_STATE::INPUT, fpcfFusionManager_, ui_, vis_));
    initTimer();
    helpWindow_ = new fpcf_gui::fpcfHelpWindow(this);
}

fpcf_gui::fpcfMainGui::~fpcfMainGui()
{
    delete visTimer_;
}

void
fpcf_gui::fpcfMainGui::initFusionManager()
{
    fpcfFusionManager_.reset(new fpcf_gui::fpcfFusionManager<PointType>());
    fpcfFusionManager_->setVoxelgridSize(ui_.doubleSpinBox_VoxelgridSize->value() / 100);
    fpcfFusionManager_->setFilterDistance((float)ui_.spinBox_FilterDistance->value() / 100);
    fpcfFusionManager_->setMaxPlaneSize(ui_.spinBox_MaxPlaneSize->value());
    fpcfFusionManager_->setOutlinerRadius(ui_.doubleSpinBox_OutlinerRadius->value() / 100);
    fpcfFusionManager_->setNormalEstimationRadius(ui_.doubleSpinBox_NormalEstRadius->value() / 100);
    fpcfFusionManager_->calculatePersistentFeature(ui_.checkBox_PersistantFeatures->isChecked());
    fpcfFusionManager_->setDescriptor(ui_.comboBox_Descriptor->currentText().toStdString());
    fpcfFusionManager_->setDescriptorRadius(ui_.doubleSpinBox_DescriptorRadius->value() / 100);
    fpcfFusionManager_->setDetector(ui_.comboBox_Detector->currentText().toStdString());
    fpcfFusionManager_->setRansacIterations(ui_.spinBox_RansacIter->value());
    fpcfFusionManager_->setPairICPIterations(ui_.spinBox_PairICPIter->value());
    fpcfFusionManager_->setGlobalLuMIterations(ui_.spinBox_GlobalLuMIter->value());
}

void
fpcf_gui::fpcfMainGui::initTimer()
{
    visTimer_ = new QTimer(this);
    connect(visTimer_, SIGNAL (timeout()), this, SLOT (updatePointClouds()));
    visTimer_->start(33); // 33 ms ~= 30 fps
}

void
fpcf_gui::fpcfMainGui::addPointCloud(fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr fpcfPointCloud)
{
    vis_->addPointCloud(fpcfPointCloud->getPointCloud(), fpcfPointCloud->getId());
    pcl::PointCloud<PointType>::Ptr keypointCloudPlaceholder(new pcl::PointCloud<PointType>());
    vis_->addPointCloud(keypointCloudPlaceholder, fpcfPointCloud->getId() + "keypoints");
}

void
fpcf_gui::fpcfMainGui::removePointCloud(fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr fpcfPointCloud)
{
    vis_->removePointCloud(fpcfPointCloud->getId());
    vis_->removePointCloud(fpcfPointCloud->getId() + "keypoints");
}

void 
fpcf_gui::fpcfMainGui::createPointCloudCopies(std::vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> &pointClouds)
{
    pointClouds.clear();
    // point clouds from files
    for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
    {
        fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr pointCloudCopy(new fpcf::fpcfPointCloudData<PointType, DescriptorType>(*pointClouds_.at(pcNr)));
        pointClouds.push_back(pointCloudCopy);
    }
    // point clouds from Kinect sensors
    for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
    {
        fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr pointCloudCopy(new fpcf::fpcfPointCloudData<PointType, DescriptorType>(*grabbedClouds_.at(pcNr)));
        pointClouds.push_back(pointCloudCopy);
    }
}

void
fpcf_gui::fpcfMainGui::updatePointClouds()
{
    if (fpcfStateMachine_->getCurrentStateName() == fpcf_gui::GUI_STATE::INPUT)
    {
        grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
        for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
        {
            vis_->updatePointCloud(grabbedClouds_.at(pcNr)->getPointCloud(), grabbedClouds_.at(pcNr)->getId());
        }
        ui_.widget_Visualizer->update();
    }
    else if (fpcfStateMachine_->getCurrentStateName() == fpcf_gui::GUI_STATE::SINGLE_FUSION)
    {
        grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
        fpcfFusionManager_->setPointClouds(grabbedClouds_);
        fpcfFusionManager_->transformPointClouds();
        for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
        {
            vis_->updatePointCloud(grabbedClouds_.at(pcNr)->getPointCloud(), grabbedClouds_.at(pcNr)->getId());
        }
        ui_.widget_Visualizer->update();
    }
    else if (fpcfStateMachine_->getCurrentStateName() == fpcf_gui::GUI_STATE::CONTINUOUS_FUSION)
    {
        ui_.toolButton_ContinuousFusion->click();
    }
}

void
fpcf_gui::fpcfMainGui::toolButton_Input_Clicked()
{
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    createPointCloudCopies(fusionCloudCopies_);
    fpcfFusionManager_->setPointClouds(fusionCloudCopies_);
    fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::INPUT);
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::toolButton_Prepared_Clicked()
{
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    createPointCloudCopies(fusionCloudCopies_);
    fpcfFusionManager_->setPointClouds(fusionCloudCopies_);
    fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::PREPARED);
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::toolButton_SingleFusion_Clicked()
{
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    createPointCloudCopies(fusionCloudCopies_);
    fpcfFusionManager_->setPointClouds(fusionCloudCopies_);
    fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::SINGLE_FUSION);
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::toolButton_ContinuousFusion_Clicked()
{
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    createPointCloudCopies(fusionCloudCopies_);
    fpcfFusionManager_->setPointClouds(fusionCloudCopies_);
    fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::CONTINUOUS_FUSION);
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::toolButton_Mesh_Clicked()
{
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    createPointCloudCopies(fusionCloudCopies_);
    fpcfFusionManager_->setPointClouds(fusionCloudCopies_);
    fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::MESH);
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::actionNew_Triggered()
{
    if (fpcfStateMachine_->getCurrentStateName() != fpcf_gui::GUI_STATE::INPUT)
    {
        fpcfStateMachine_->changeState(fpcf_gui::GUI_STATE::INPUT);
    }
    pushButton_Stop_Clicked();
    pushButton_RemoveAllPointClouds_Clicked();
}

void
fpcf_gui::fpcfMainGui::actionAbout_Triggered()
{
    QMessageBox::about(this, "About FPCF GUI", "FPCF GUI created 2013 by Thomas Weber");
}

void
fpcf_gui::fpcfMainGui::actionHelp_Triggered()
{
    helpWindow_->show();
}

void
fpcf_gui::fpcfMainGui::actionQuit_Triggered()
{
    close();
}

void
fpcf_gui::fpcfMainGui::actionSaveAs_Triggered()
{
    if (fpcfStateMachine_->getCurrentStateName() == fpcf_gui::GUI_STATE::MESH && fusionCloudCopies_.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save Mesh", "", "Stanford(*.ply)");
        fpcf_gui::fpcfMeshState<PointType>::Ptr meshState = boost::dynamic_pointer_cast<fpcf_gui::fpcfMeshState<PointType>>(fpcfStateMachine_->getCurrentState());
        pcl::io::savePLYFileBinary(fileName.toStdString(), *meshState->getMesh());
    }
    else if (fpcfStateMachine_->getCurrentStateName() == fpcf_gui::GUI_STATE::PREPARED && fusionCloudCopies_.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save Point Cloud", "", "Point Cloud Data Files(*.pcd)");
        pcl::PointCloud<PointType> concatPointCloud;
        for (int pcNr = 0; pcNr < fusionCloudCopies_.size(); pcNr++)
        {
            concatPointCloud += *fusionCloudCopies_.at(pcNr)->getMostDownsampledPointCloud()->getPointCloud();
        }
        pcl::io::savePCDFileBinary<PointType>(fileName.toStdString(), concatPointCloud);
    }
    else if (fusionCloudCopies_.size() > 0)
    {
        QString fileName = QFileDialog::getSaveFileName(this, "Save Point Cloud", "", "Point Cloud Data Files(*.pcd)");
        pcl::PointCloud<PointType> concatPointCloud;
        for (int pcNr = 0; pcNr < fusionCloudCopies_.size(); pcNr++)
        {
            concatPointCloud += *fusionCloudCopies_.at(pcNr)->getPointCloud();
        }
        pcl::io::savePCDFileBinary<PointType>(fileName.toStdString(), concatPointCloud);
    }
    else
    {
        QMessageBox::about(this, "Nothing to save", "There is no data, which could be saved!");
    }
}

void
fpcf_gui::fpcfMainGui::checkBox_PersistantFeatures_StateChanged(int i)
{
    fpcfFusionManager_->calculatePersistentFeature(i);
}


void
fpcf_gui::fpcfMainGui::comboBox_Color_CurrentIndexChanged(QString qString)
{
    std::string color = qString.toStdString();
    if (color == "normal")
    {
        vis_->setPointCloudColor(fpcf::COLOR_STATE::NORMAL);
    }
    else if (color == "random")
    {
        vis_->setPointCloudColor(fpcf::COLOR_STATE::RANDOM);
    }
}

void
fpcf_gui::fpcfMainGui::comboBox_Descriptor_CurrentIndexChanged(QString qString)
{
    fpcfFusionManager_->setDescriptor(qString.toStdString());
}

void
fpcf_gui::fpcfMainGui::comboBox_Detector_CurrentIndexChanged(QString qString)
{
    fpcfFusionManager_->setDetector(qString.toStdString());
}

void
fpcf_gui::fpcfMainGui::doubleSpinBox_DescriptorRadius_ValueChanged(double d)
{
    fpcfFusionManager_->setDescriptorRadius(d / 100);
}

void
fpcf_gui::fpcfMainGui::doubleSpinBox_NormalEstRadius_ValueChanged(double d)
{
    fpcfFusionManager_->setNormalEstimationRadius(d / 100);
}

void
fpcf_gui::fpcfMainGui::doubleSpinBox_OutlinerRadius_ValueChanged(double d)
{
    fpcfFusionManager_->setOutlinerRadius(d / 100);
}

void
fpcf_gui::fpcfMainGui::doubleSpinBox_VoxelgridSize_ValueChanged(double d)
{
    fpcfFusionManager_->setVoxelgridSize(d / 100);
}

void
fpcf_gui::fpcfMainGui::pushButton_AddPointCloud_Clicked()
{
    fpcf::fpcfPointCloudReader<PointType, DescriptorType> reader;
    QStringList fileNames = QFileDialog::getOpenFileNames(this, "Add Point Clouds", "", "Point Cloud Data Files(*.pcd)");
    for (int i = 0; i < fileNames.size(); i++)
    {
        std::string id = "pcNr" + boost::lexical_cast<std::string>(pointClouds_.size());
        fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr newPointCloud(new fpcf::fpcfPointCloudData<PointType, DescriptorType>(id));
        reader.loadPCDFile("", fileNames.at(i).toStdString(), newPointCloud);
        addPointCloud(newPointCloud);
        pointClouds_.push_back(newPointCloud);
    }
    ui_.toolButton_Input->click();
}

void
fpcf_gui::fpcfMainGui::pushButton_CaptureInput_Clicked()
{
    for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
    {
        grabbedClouds_.at(pcNr)->setId(grabbedClouds_.at(pcNr)->getId() + boost::lexical_cast<std::string>(pointClouds_.size()));
        addPointCloud(grabbedClouds_.at(pcNr));
        pointClouds_.push_back(grabbedClouds_.at(pcNr));
    }
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::pushButton_Start_Clicked()
{
    visTimer_->stop();
    multiKinectGrabber_.removeAllKinectDevices();
    for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
    {
        removePointCloud(grabbedClouds_.at(pcNr));
    }
    ui_.label_KinectSensorInitialization->show();
    ui_.label_KinectSensorInitialization->repaint();
    // init Kinect devices
    int nrDevices = ui_.spinBox_NrDevices->value();
    for (int deviceNr = 1; deviceNr <= nrDevices; deviceNr++)
    {
        multiKinectGrabber_.addKinectDevice("#" + boost::lexical_cast<std::string>(deviceNr));
    }
    // init pointClouds
    multiKinectGrabber_.start();
    grabbedClouds_ = multiKinectGrabber_.getGrabbedPointClouds();
    for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
    {
        addPointCloud(grabbedClouds_.at(pcNr));
    }
    ui_.label_KinectSensorInitialization->hide();
    ui_.label_KinectSensorInitialization->repaint();
    ui_.toolButton_Input->click();
    visTimer_->start(33); // 33 ms ~= 30 fps
}

void
fpcf_gui::fpcfMainGui::pushButton_Stop_Clicked()
{
    visTimer_->stop();
    multiKinectGrabber_.removeAllKinectDevices();
    for (int pcNr = 0; pcNr < grabbedClouds_.size(); pcNr++)
    {
        removePointCloud(grabbedClouds_.at(pcNr));
    }
    grabbedClouds_.clear();
    visTimer_->start();
    ui_.widget_Visualizer->update();
}

void
fpcf_gui::fpcfMainGui::pushButton_RemoveAllPointClouds_Clicked()
{
    for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
    {
        removePointCloud(pointClouds_.at(pcNr));
    }
    pointClouds_.clear();
}

void
fpcf_gui::fpcfMainGui::spinBox_FilterDistance_ValueChanged(int i)
{
    fpcfFusionManager_->setFilterDistance((float)i / 100);
}

void
fpcf_gui::fpcfMainGui::spinBox_GlobalLuMIter_ValueChanged(int i)
{
    fpcfFusionManager_->setGlobalLuMIterations(i);
}

void
fpcf_gui::fpcfMainGui::spinBox_MaxPlaneSize_ValueChanged(int i)
{
    fpcfFusionManager_->setMaxPlaneSize(i);
}

void
fpcf_gui::fpcfMainGui::spinBox_PairICPIter_ValueChanged(int i)
{
    fpcfFusionManager_->setPairICPIterations(i);
}

void fpcf_gui::fpcfMainGui::spinBox_RansacIter_ValueChanged(int i)
{
    fpcfFusionManager_->setRansacIterations(i);
}
