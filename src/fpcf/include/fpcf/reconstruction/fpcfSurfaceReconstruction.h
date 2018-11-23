/**
 * @file    fpcfSurfaceReconstruction.h
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

#ifndef _FPCFSURFACERECONSTRUCTION_H_
#define _FPCFSURFACERECONSTRUCTION_H_

// STL

// Boost

// PCL
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * This class performs a surface reconstruction of point cloud using the 
     * greedy projection triangulation. Beforehand, a voxelgrid filter and a 
     * noise filter is applied to reduce the noise of the point cloud. 
     * Afterwards the resulting mesh is smoothed using a passband filter.
     *
     * see Paper: 
     *   Gopi, M. & Krishnan, S.
     *   "A fast and efficient projection-based approach for surface 
     *   reconstruction"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfSurfaceReconstruction
    {
        
        public:
            /**
             * Constructor that creates a new empty fpcfSurfaceReconstruction 
             * object.
             */
            fpcfSurfaceReconstruction();

            /**
             * Destructor
             */
            virtual ~fpcfSurfaceReconstruction();

            /**
             * Setter for the fpcfPointCloud, which will be reconstructed.
             *
             * @param[in] fpcfPointCloud the point cloud for the reconstruction
             */
            void setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);

            /**
             * Setter for the size of the voxelgrid during the preparation of 
             * the point cloud. 
             *
             * @param[in] size the size of the voxelgrid
             */
            void setVoxelgridSize(float size);

            /**
             * Setter for the used radius of the noise filter during the 
             * preparation of the point cloud.
             *
             * @param[in] size radius of the noise filter
             */
            void setOutlinerRadius(float size);

            /**
             * Setter used passband value to smooth the mesh after the surface 
             * reconstruction. Lower values lead to a stronger smoothing 
             * effect.
             *
             * @param[in] value the passband value
             */
            void setPassbandValue(float value);

            /**
             * Flag, if the point cloud colors should be transfered to the 
             * vertices of the resulting triangle mesh. If the point cloud 
             * contains no color information, this should be set to 'false'.
             *
             * @param[in] state if true, point cloud colors are transferred to 
             *                  mesh
             */
            void transferPointColors(bool state);

            /**
             * Getter of the current fpcfPointCloud
             *
             * @result the current fpcfPointCloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getPointCloud();

            /**
             * Getter for the computed triangle mesh. This is null, if 
             * reconstruct() wasn't invoked beforehand.
             *
             * @result the computed triangle mesh after the reconstruction
             */
            pcl::PolygonMesh::Ptr  getTriangleMesh();

            /**
             * Performs the reconstruction of the point cloud. A copy of the 
             * point cloud is prepared beforehand and prepared to reduce the 
             * noise. The resulting triangle mesh can be accessed using the
             * getTriangleMesh() method.
             */
            void reconstruct();
        
        protected:
            
        private:
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedyTriangulation_;
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>::Ptr normalEstimator_;
            pcl::VoxelGrid<PointT> downsampling_;
            pcl::RadiusOutlierRemoval<PointT> noiseFilter_;
            pcl::MeshSmoothingWindowedSincVTK smoothing_;
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud_;
            typename pcl::PointCloud<PointT>::Ptr pclPreparedCopy_;
            pcl::PointCloud<pcl::PointNormal>::Ptr pclPointNormalCopy_;
            pcl::PolygonMesh::Ptr triangleMesh_;
            bool transfertPointColors_;

            void createPreparedCopy();
            void createPointNormalCopy();
            void createTriangleMesh();
            void applyMeshSmoothing();

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::fpcfSurfaceReconstruction()
    {
        triangleMesh_.reset(new pcl::PolygonMesh());
        greedyTriangulation_.setSearchRadius(0.05);
        greedyTriangulation_.setMu(2.5);
        greedyTriangulation_.setMaximumNearestNeighbors(100);
        greedyTriangulation_.setNormalConsistency(false);
        int nrCores = boost::thread::hardware_concurrency();
        normalEstimator_.reset(new pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>(nrCores));
        normalEstimator_->setKSearch(20);
        smoothing_.setPassBand(0.075f);
        downsampling_.setLeafSize(0.0075f, 0.0075f, 0.0075f);
        noiseFilter_.setMinNeighborsInRadius(2);
        noiseFilter_.setRadiusSearch(0.01f);
        transfertPointColors_ = false;
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::~fpcfSurfaceReconstruction()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::setPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfPointCloud_ = fpcfPointCloud;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::setVoxelgridSize(float size)
    {
        downsampling_.setLeafSize(size, size, size);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::setOutlinerRadius(float size)
    {
        noiseFilter_.setRadiusSearch(size);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::setPassbandValue(float value)
    {
        smoothing_.setPassBand(value);
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::getPointCloud()
    {
        return fpcfPointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    pcl::PolygonMesh::Ptr 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::getTriangleMesh()
    {
        return triangleMesh_;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::transferPointColors(bool state)
    {
        transfertPointColors_ = state;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::createPreparedCopy()
    {
        pclPreparedCopy_.reset(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr downsampledCloud(new pcl::PointCloud<PointT>());
        // downsampling
        if (downsampling_.getLeafSize()[0] > 0.0001)
        {
            downsampling_.setInputCloud(fpcfPointCloud_->getPointCloud());
            downsampling_.filter(*downsampledCloud);
            std::cout << "#downsampled points for surface reconstruction: " << downsampledCloud->width * downsampledCloud->height << std::endl;
        }
        else
        {
            downsampledCloud.reset(new pcl::PointCloud<PointT>(*fpcfPointCloud_->getPointCloud()));
            std::cout << "mesh voxelgridsize too small, skipping voxelgrid downsampling " << std::endl;
        }
        // noise filtering
        if (noiseFilter_.getRadiusSearch() > 0.0001)
        {
            noiseFilter_.setInputCloud(downsampledCloud);
            noiseFilter_.filter(*pclPreparedCopy_);
            std::cout << "#noise filtered points for surface reconstruction: " << pclPreparedCopy_->width * pclPreparedCopy_->height << std::endl;
        }
        else
        {
            pclPreparedCopy_ = downsampledCloud;
            std::cout << "mesh outliner radius filter too small, skipping noise filtering" << std::endl;
        }
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::createPointNormalCopy()
    {
        // extract XYZ values of the input point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud<PointT, pcl::PointXYZ>(*pclPreparedCopy_, *pclPointCloudXYZ);
        // estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        normalEstimator_->setInputCloud (pclPointCloudXYZ);
        normalEstimator_->compute (*normals);
        // merge XYZ and normal values
        pclPointNormalCopy_.reset(new pcl::PointCloud<pcl::PointNormal>());
        pcl::concatenateFields (*pclPointCloudXYZ, *normals, *pclPointNormalCopy_);
    }

        template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::createTriangleMesh()
    {
        greedyTriangulation_.setInputCloud(pclPointNormalCopy_);
        greedyTriangulation_.reconstruct(*triangleMesh_);
        if (transfertPointColors_)
        {
            // change to old point cloud
            pcl::toPCLPointCloud2(*pclPreparedCopy_, triangleMesh_->cloud);
        }
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::applyMeshSmoothing()
    {
        if (smoothing_.getPassBand() > 0.0001)
        {
            smoothing_.setInputMesh(triangleMesh_);
            triangleMesh_.reset(new pcl::PolygonMesh());
            smoothing_.process(*triangleMesh_);
        }
        else
        {
            std::cout << "passband value too small, skipping passband smoothing" << std::endl;
        }
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfSurfaceReconstruction<PointT, DescriptorT>::reconstruct()
    {
        createPreparedCopy();
        createPointNormalCopy();
        createTriangleMesh();
        applyMeshSmoothing();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFSURFACERECONSTRUCTION_H_