/**
 * @file    fpcfLuMGlobalAlignment.h
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

#ifndef _FPCFLUMGLOBALALIGNMENT_H_
#define _FPCFLUMGLOBALALIGNMENT_H_

// STL

// Boost
#include <boost/thread.hpp>

// PCL
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/registration/graph/fpcfCloudGraph.h>

namespace fpcf
{

    /**
     * This optimized the position of all input point clouds at once using a 
     * Lu-Milos optimization.
     *
     * see paper:
     *   Lu, F. & Milios, E.
     *   "Globally Consistent Range Scan Alignment for Environment Mapping"
     *
     *   Borrmann, D.; Elseberg, J.; Lingemann, K.; Nuechter, A. & Hertzberg, J.
     *   "Globally consistent 3D mapping with scan matching"
     */
    template <typename PointT, typename DescriptorT>
    class fpcfLuMGlobalAlignment
    {
        public:
            typedef boost::shared_ptr<fpcfLuMGlobalAlignment<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new empty fpcfLuMGlobalAlignment 
             * object
             */
            fpcfLuMGlobalAlignment();

            /**
             * Destructor
             */
            virtual ~fpcfLuMGlobalAlignment();

            /**
             * Sets the vector of point clouds which will be optimized in their positions
             *
             * @param[in] fpcfPointClouds the point clouds of the global 
             *                           alignment
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Sets the predecessor map, which contains information about the 
             * best point cloud pairs. This map needs to be computed by the 
             * fpcfGraphOptimization beforehand and is used to find the nearest 
             * point neighbors between the best point cloud pairs during the 
             * global alignment. 
             *
             * @param[in] predecessorMap the predecessor map of the 
             *                       fpcfGraphOptimization using the input 
             *                       vector of point clouds
             */
            void setPredecessorMap(std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> predecessorMap);

            /**
             * Sets the number of iterations of the global alignment
             *
             * @param[in] iterations the number of iterations
             */
            void setIterations(int iterations);

            /**
             * Getter for the resulting transformations to optimize the 
             * alignment of all input point clouds. The resulting map contains 
             * the id of the point cloud and the according transformation 
             * matrix. The map contains only meaningful values, if 
             * performAlignment() was invoked beforehand.
             *
             * @returns a map containing the point cloud id and the according 
             *          transformation
             */
            std::map<std::string, Eigen::Matrix4f> getTransformationMap();

            /**
             * Performs the actual global optimization of the alignment of the 
             * input point cloud. This optimization uses automatically the 
             * most downsampled version of the input point clouds and directly 
             * modifies the positions of the input point clouds! The resulting 
             * transformation map can be accessed using the 
             * getTransformationMap() method.
             */
            void performAlignment();

        protected:

        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds_;
            std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> predecessorMap_;
            pcl::registration::LUM<PointT> *lum_;
            boost::mutex lumMutex_;
            std::map<std::string, Eigen::Matrix4f> transformationMap_;
            int iterations_;

            void init();
            void initTransformationMap();
            void calculateCorrespondences(int sourceNr, int targetNr);
            
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::fpcfLuMGlobalAlignment()
    {
        init();
        initTransformationMap();
    }
    
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::~fpcfLuMGlobalAlignment()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        pointClouds_ = fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::setPredecessorMap(std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> predecessorMap)
    {
        predecessorMap_ = predecessorMap;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::setIterations(int iterations)
    {
        iterations_ = iterations;
    }

    template <typename PointT, typename DescriptorT>
    std::map<std::string, Eigen::Matrix4f>
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::getTransformationMap()
    {
        return transformationMap_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::performAlignment()
    {
        initTransformationMap();
        lum_ = new pcl::registration::LUM<PointT>();
        lum_->setMaxIterations(1);
        lum_->setConvergenceThreshold(0.001f);
        transformationMap_.clear();
        for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
        {
            lum_->addPointCloud(pointClouds_.at(pcNr)->getMostDownsampledPointCloud()->getPointCloud());
        }
        std::cout << "begin LuM with #iters: " << iterations_ << std::endl;
        // compute transformation
        for (int it = 0; it < iterations_; it++)
        {
            for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor sourceNr = 0; sourceNr < pointClouds_.size(); sourceNr++)
            {
                // parallelized
                boost::thread_group thrdGrp;
                for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor targetNr = 0; targetNr < pointClouds_.size(); targetNr++)
                {
                    if (sourceNr != targetNr && targetNr == predecessorMap_[sourceNr]) 
                    {
                        thrdGrp.create_thread(boost::bind(&fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::calculateCorrespondences, this, sourceNr, targetNr));
                    }
                }
                thrdGrp.join_all();
            }
            cout << "begin lum iter: " << it << endl;
            lum_->compute();
            for(int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
            {
                pointClouds_.at(pcNr)->getMostDownsampledPointCloud()->setPointCloud(lum_->getTransformedCloud(pcNr));
            }
        }
        // transform point clouds
        for(int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
        {
            fpcf::fpcfPointCloudTransformation<PointT, DescriptorT> fpcfPointCloudTransformation(pointClouds_.at(pcNr));
            fpcfPointCloudTransformation.setTransformation(lum_->getTransformation(pcNr).matrix());
            fpcfPointCloudTransformation.transformPointCloud();
            transformationMap_.erase(pointClouds_.at(pcNr)->getId());
            transformationMap_.insert(std::pair<std::string, Eigen::Matrix4f>(pointClouds_.at(pcNr)->getId(), lum_->getTransformation(pcNr).matrix()));
        }
        delete lum_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::init()
    {
        const int DEFAULT_ITERATIONS = 3;
        setIterations(DEFAULT_ITERATIONS);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::initTransformationMap()
    {
        transformationMap_.clear();
        for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
        {
            transformationMap_.insert(std::pair<std::string, Eigen::Matrix4f>(pointClouds_.at(pcNr)->getId(), Eigen::Matrix4f::Identity()));
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::calculateCorrespondences(int sourceNr, int targetNr)
    {
        double corresDist = 0.10;
        pcl::registration::CorrespondenceEstimation<PointT, PointT> corrEst;
        corrEst.setInputTarget(pointClouds_.at(targetNr)->getMostDownsampledPointCloud()->getPointCloud());
        corrEst.setInputSource(pointClouds_.at(sourceNr)->getMostDownsampledPointCloud()->getPointCloud());
        pcl::CorrespondencesPtr corres(new pcl::Correspondences);
        corrEst.determineReciprocalCorrespondences(*corres, corresDist);
        if (corres->size() > 2)
        {
            lumMutex_.lock();
            lum_->setCorrespondences(sourceNr, targetNr, corres);
            lumMutex_.unlock();
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFLUMGLOBALALIGNMENT_H_