/**
 * @file    fpcfMeshState.h
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

#ifndef _FPCFMESHSTATE_H_
#define _FPCFMESHSTATE_H_

// STL

// Boost

// Qt

// PCL

// FPCF
#include <fpcf_gui/control/guiStates/fpcfState.h>
#include <fpcf/reconstruction/fpcfSurfaceReconstruction.h>
#include <fpcf/tools/fpcfPointCloudConcatenator.h>

namespace fpcf_gui
{
    /**
     * This state performs a point cloud fusion and a surface reconstruction of
     * the input point clouds. The resulting mesh is added to the viewer.
     */
    template <typename PointT>
    class fpcfMeshState : public fpcfState<PointT>
    {
        
        public:
            typedef boost::shared_ptr<fpcfMeshState<PointT>> Ptr;

            /**
             * Constructor that creates a new mesh state.
             *
             * @param[in] fpcfFusionManager the fusion manager to fuse the point
             *                             clouds
             * @param[in] ui the used fpcf ui 
             * @param[in] vis the visualizer to render the resulting point 
             *                clouds
             */
            fpcfMeshState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis);
            
            /**
             * Destructor
             */
            virtual ~fpcfMeshState();

            /**
             * Getter for the resulting triangle mesh after a surface 
             * reconstruction.
             */
            pcl::PolygonMesh::Ptr getMesh();

            /*
             * Performs a point cloud fusion of all input point clouds and a 
             * surface reconstruction. The resulting triangle mesh is added to
             * the visualization.
             */
            void onEnter();

            /**
             * Only performs a surface reconstruction of the fused input clouds
             * and adds the resulting triangle mesh to the visualizer.
             */
            void onUpdate();

            /**
             * Removes the triangle mesh from the visualization.
             */
            void onExit();


        protected:

        private:
            std::string meshId_;
            pcl::PolygonMesh::Ptr triangleMesh_;

            void createMesh();

            // hide default constructor
            fpcfMeshState();

    };

    template <typename PointT>
    fpcf_gui::fpcfMeshState<PointT>::fpcfMeshState(typename fpcf_gui::fpcfFusionManager<PointT>::Ptr fpcfFusionManager, Ui::MainGui ui, typename fpcf::fpcfVisualizer<PointT, DefaultDescriptorType>::Ptr vis)
        : fpcfState(fpcf_gui::GUI_STATE::MESH, fpcfFusionManager, ui, vis)
    {
        meshId_ = "mesh";
    }
            
    template <typename PointT>
    fpcf_gui::fpcfMeshState<PointT>::~fpcfMeshState()
    {
        
    }

    template <typename PointT>
    pcl::PolygonMesh::Ptr 
    fpcf_gui::fpcfMeshState<PointT>::getMesh()
    {
        return triangleMesh_;
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfMeshState<PointT>::onEnter()
    {
        hidePointClouds();
        fpcfFusionManager_->performFusion();
        fpcfPointClouds_ = fpcfFusionManager_->getPointClouds();
        if (fpcfPointClouds_.size() > 0)
        {
            createMesh();
            vis_->addPolygonMesh(*triangleMesh_, meshId_);
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfMeshState<PointT>::onUpdate()
    {
        if (fpcfPointClouds_.size() > 0)
        {
            createMesh();
            vis_->removePolygonMesh(meshId_);
            vis_->addPolygonMesh(*triangleMesh_, meshId_);
        }
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfMeshState<PointT>::onExit()
    {
        vis_->removePolygonMesh(meshId_);
    }

    template <typename PointT>
    void 
    fpcf_gui::fpcfMeshState<PointT>::createMesh()
    {
        fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>::Ptr fpcfPointCloudConcat(new fpcf::fpcfPointCloudData<PointT, DefaultDescriptorType>("fpcfPointCloudConcat"));
        fpcf::fpcfPointCloudConcatenator<PointT, DefaultDescriptorType> fpcfPointCloudConcatenator;
        fpcfPointCloudConcatenator.setPointClouds(fpcfPointClouds_);
        fpcfPointCloudConcatenator.concat(fpcfPointCloudConcat);
        fpcf::fpcfSurfaceReconstruction<PointType, DefaultDescriptorType> surfaceReconstruction;
        surfaceReconstruction.setVoxelgridSize(ui_.doubleSpinBox_MeshVoxelgridSize->value() / 100);
        surfaceReconstruction.setOutlinerRadius(ui_.doubleSpinBox_MeshOutlinerRadius->value() / 100);
        surfaceReconstruction.setPassbandValue(ui_.doubleSpinBox_PassbandValue->value());
        surfaceReconstruction.transferPointColors(ui_.checkBox_TransferPointColors->isChecked());
        surfaceReconstruction.setPointCloud(fpcfPointCloudConcat);
        surfaceReconstruction.reconstruct();
        triangleMesh_ = surfaceReconstruction.getTriangleMesh();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFMESHSTATE_H_