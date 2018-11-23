/**
 * @file    fpcfVisualizer.h
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

#ifndef _FPCFVISUALIZER_H_
#define _FPCFVISUALIZER_H_

// STL
#include <stdlib.h>
#include <string>

// Boost

// PCL
#include <pcl/visualization/pcl_visualizer.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{
    /**
     * Enumeration to define the rendering color of the point clouds.
     */
    enum COLOR_STATE { NORMAL, RANDOM };

    /**
     * This class contains methods to visualize fpcfPointClouds, normals, 
     * correspondences and meshes.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfVisualizer : public pcl::visualization::PCLVisualizer
    {
        public:
            typedef boost::shared_ptr<fpcfVisualizer<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfVisualizer
             *
             * @param[in] createInteractor if true (default), create an 
             *                             interactor, false otherwise
             */
            fpcfVisualizer(bool createInteractor = true);

            /**
             * Destructor
             */
            virtual ~fpcfVisualizer();

            /**
             * Sets the rendering color of the input point clouds.
             *
             * @param[in] color can be NORMAL or RANDOM
             */
            void setPointCloudColor(COLOR_STATE color);

            /**
             * Adds a coordinate system to the origin.
             */
            void addCoordinateSystem();

            /**
             * Adds a pcl point cloud with a specified id to the render output.
             *
             * @param[in] pclPointCloud the pcl point cloud, which will be 
             *                          added to the render output
             * @param[in] id the id of the rendered pcl point cloud
             */
            void addPointCloud(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id);

            /**
             * Adds a fpcf point cloud to the render output.
             *
             * @param[in] pointCloud the fpcf point cloud, which will be added 
             *                       to the render output
             */
            void addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Adds the normals of a fpcf point cloud to the render output.
             *
             * @param[in] pointCloud a fpcf point cloud with normals, which be
             *                       added to the render output
             */
            void addNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Adds a the keypoints of fpcf point cloud to the render output.
             *
             * @param[in] pointCloud the fpcf point cloud with keypoints, which
             *                       will be added to the render output
             */
            void addKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Adds the correspondences between all points of a fpcf point cloud
             * pair to the render output
             *
             * @param[in] pointCloudPair a point cloud pair with 
             *                           correspondences between all points
             */
            void addAllPointCorrespondences(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Adds the correspondences between the keypoints of a fpcf point 
             * cloud pair to the render output
             *
             * @param[in] pointCloudPair a point cloud pair with 
             *                           correspondences between the keypoints
             */
            void addKeypointCorrespondences(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair);

            /**
             * Adds a polygon mesh to the render output
             *
             * @param[in] mesh the polygon mesh itself
             * @param[in] id the id for the rendered polygon mesh
             */
            void addPolygonMesh(pcl::PolygonMesh mesh, std::string id);

            /**
             * Updates the render output of an already added pcl point cloud.
             *
             * @param[in] pclPointCloud the pcl point cloud with the updated 
             *                          points
             * @param[in] id the id of the already added point cloud
             */
            void updatePointCloud(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id);

            /**
             * Updates the render output of an already added fpcf point cloud.
             *
             * @param[in] pointCloud the fpcf point cloud with the updated 
             *                       points
             */
            void updatePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Updates the keypoint render output of an already added fpcf point
             * cloud.
             *
             * @param[in] pointCloud the fpcf point cloud with the updated 
             *                       keypoints
             */
            void updateKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

            /**
             * Adds a keyboard callback to change the rendering colors of the 
             * point clouds.
             */
            void registerColorCallback();

            /**
             * Opens a rendering window, showing all added point clouds, 
             * normals, correspondences and meshes
             */
            void visualize();

        protected:

        private:

            COLOR_STATE currentColor_;
            std::map<std::string, typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr> colorHandlerMap_;

            void addCorrespondences(typename pcl::PointCloud<PointT>::Ptr pclPointCloudSource, typename pcl::PointCloud<PointT>::Ptr pclPointCloudTarget, pcl::CorrespondencesPtr correspondences);
            
            typename pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr createColorHandler(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id);
            static void colorCallback(const pcl::visualization::KeyboardEvent &event, void *colorState)
            {
                if(event.getKeySym() == "F1")
                {
                    *(static_cast<COLOR_STATE*>(colorState)) = COLOR_STATE::NORMAL;
                }
                else if(event.getKeySym() == "F2")
                {
                    *(static_cast<COLOR_STATE*>(colorState)) = COLOR_STATE::RANDOM;
                }
            }

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfVisualizer<PointT, DescriptorT>::fpcfVisualizer(bool createInteractor = true)
        : PCLVisualizer("Point Cloud Fusion", createInteractor)
    {
        PCLVisualizer::initCameraParameters();
        currentColor_ = COLOR_STATE::NORMAL;
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfVisualizer<PointT, DescriptorT>::~fpcfVisualizer()
    {
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfVisualizer<PointT, DescriptorT>::setPointCloudColor(COLOR_STATE color)
    {
        currentColor_ = color;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfVisualizer<PointT, DescriptorT>::addCoordinateSystem()
    {
        PCLVisualizer::addCoordinateSystem(1.0);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        addPointCloud(pointCloud->getPointCloud(), pointCloud->getId());
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcfVisualizer<PointT, DescriptorT>::addNormalCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        PCLVisualizer::addPointCloudNormals<PointT, pcl::Normal>(pointCloud->getPointCloud(), pointCloud->getNormalsCloud(), 10, 0.05f, pointCloud->getId() + "normals");
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        const double POINT_SIZE = 7;
        std::string id = pointCloud->getId() + "keypoints";
        typename pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr pointCloudColorHandler = createColorHandler(pointCloud->getKeypointCloud(), id);
        PCLVisualizer::addPointCloud(pointCloud->getKeypointCloud(), *pointCloudColorHandler, id);
        PCLVisualizer::setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, id);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addAllPointCorrespondences(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        addCorrespondences(pointCloudPair->getPointCloudSource()->getPointCloud(), pointCloudPair->getPointCloudTarget()->getPointCloud(), pointCloudPair->getAllPointCorrespondences());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addKeypointCorrespondences(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr pointCloudPair)
    {
        addCorrespondences(pointCloudPair->getPointCloudSource()->getKeypointCloud(), pointCloudPair->getPointCloudTarget()->getKeypointCloud(), pointCloudPair->getKeypointCorrespondences());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addCorrespondences(typename pcl::PointCloud<PointT>::Ptr pclPointCloudSource, typename pcl::PointCloud<PointT>::Ptr pclPointCloudTarget, pcl::CorrespondencesPtr correspondences)
    {
        for (int i = 0; i < correspondences->size(); i+=1)
        {
            char idBuf [32];
            int idBase = 10;
            _itoa(i, idBuf, idBase);
            PointT p1 = pclPointCloudSource->at(correspondences->at(i).index_query);
            PointT p2 = pclPointCloudTarget->at(correspondences->at(i).index_match);
            PCLVisualizer::addLine(p1, p2, 0.7, 0.17, 0.17, idBuf);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addPolygonMesh(pcl::PolygonMesh mesh, std::string id)
    {
        PCLVisualizer::addPolygonMesh(mesh, id); 
        getRendererCollection()->GetFirstRenderer()->GetActors()->GetLastActor()->GetProperty()->SetInterpolationToPhong();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::updatePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        updatePointCloud(pointCloud->getPointCloud(), pointCloud->getId());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::updateKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        updatePointCloud(pointCloud->getKeypointCloud(), pointCloud->getId() + "keypoints");
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::visualize()
    {
        while (!wasStopped())
        {
          PCLVisualizer::spinOnce(100);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::addPointCloud(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id)
    {
        typename pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr pointCloudColorHandler = createColorHandler(pclPointCloud, id);
        if (currentColor_ == COLOR_STATE::NORMAL)
        {
            PCLVisualizer::addPointCloud(pclPointCloud, id);
        }
        else if (currentColor_ == COLOR_STATE::RANDOM)
        {
            PCLVisualizer::addPointCloud(pclPointCloud, *pointCloudColorHandler, id);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::updatePointCloud(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id)
    {
        if (currentColor_ == COLOR_STATE::NORMAL)
        {
            PCLVisualizer::updatePointCloud(pclPointCloud, id);
        }
        else if (currentColor_ == COLOR_STATE::RANDOM)
        {
            typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr pointCloudColorHandler = colorHandlerMap_.find(id)->second;
            PCLVisualizer::updatePointCloud(pclPointCloud, *pointCloudColorHandler, id);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfVisualizer<PointT, DescriptorT>::registerColorCallback()
    {
        PCLVisualizer::registerKeyboardCallback(&fpcfVisualizer<PointT, DescriptorT>::colorCallback, (void*)&currentColor_);
    }

    template <typename PointT, typename DescriptorT>
    typename pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr
    fpcfVisualizer<PointT, DescriptorT>::createColorHandler(typename pcl::PointCloud<PointT>::Ptr pclPointCloud, std::string id)
    {
        double r = rand() % 255;
        double g = rand() % 255;
        double b = rand() % 255;
        typename pcl::visualization::PointCloudColorHandlerCustom<PointT>::Ptr pointCloudColorHandler(new pcl::visualization::PointCloudColorHandlerCustom<PointT>(pclPointCloud, r, g, b));
        colorHandlerMap_.insert(std::pair<std::string, typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr>(id, pointCloudColorHandler));
        return pointCloudColorHandler;
    }
    
} // end namespace fpcf

#endif // #ifndef _FPCFVISUALIZER_H_