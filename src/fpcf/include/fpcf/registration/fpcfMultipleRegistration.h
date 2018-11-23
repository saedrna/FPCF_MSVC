/**
 * @file    fpcfMultipleRegistration.h
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

#ifndef _FPCFMULTIPLEREGISTRATION_H_
#define _FPCFMULTIPLEREGISTRATION_H_

// STL

// Boost
#include <boost/thread.hpp>

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/data/fpcfPointCloudDataPair.h>
#include <fpcf/registration/fpcfPointCloudPreparation.h>
#include <fpcf/registration/fpcfPairRegistration.h>
#include <fpcf/registration/graph/fpcfGraphOptimization.h>
#include <fpcf/registration/transformation/fpcfLuMGlobalAlignment.h>
#include <fpcf/registration/transformation/fpcfPointCloudTransformation.h>

namespace fpcf
{
    /**
     * This class performs all steps to perform a multiple point cloud 
     * registration. All point clouds will be prepared and a global point cloud
     * graph is used to estimate the best point cloud pairs. The point cloud 
     * pairs are aligned using a feature-based alignment and the iterative 
     * closest point algorithm. The accumalated error after the successive 
     * pairwise alignment is reduced during a global alignment step at the end.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfMultipleRegistration
    {        
        public:

            /**
             * Constructor that creates a new fpcfMultipleRegistration with a 
             * specified point cloud preparator. The correspondence rejector of
             * the feature-based alignment, the iterative closest point 
             * algorithm and the LuM global alignment algorithm are preset with
             * default values.
             *
             * @param[in] fpcfPointCloudPreparator the used point cloud 
             *                                    preparator of the multiple 
             *                                    registration
             */
            fpcfMultipleRegistration(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator);

            /**
             * Destructor
             */
            virtual ~fpcfMultipleRegistration();

            /**
             * Adds a new point cloud, which will be used during the multiple 
             * registration
             *
             * @param[in] fpcfPointCloud the new point cloud for the multiple 
             *                          registration
             */
            void addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);

            /**
             * Adds a vector of point clouds, which will be used during the 
             * multiple registration
             *
             * @param[in] fpcfPointClouds the new point clouds for the multiple 
             *                           registration
             */
            void addPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Sets a vector of point clouds, which will be used during the 
             * multiple registration
             *
             * @param[in] fpcfPointClouds the point clouds for the multiple 
             *                           registration
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Sets the used point cloud preparator during the multiple 
             * registration of the point clouds.
             *
             * @param[in] fpcfPointCloudPreparator the preparator to prepare the
             *                                    input point clouds
             */
            void setPointCloudPreparator(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator);

            /**
             * Sets the used correspondence rejector during the multiple 
             * registration of the point clouds. A default correspondence 
             * rejector is set at the instantiation of the 
             * fpcfMultipleRegistration object and can be changed by using this
             * method.
             *
             * @param[in] correspondenceRejector the preparator to prepare the
             *                                   input point clouds
             */
            void setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr correspondenceRejector);

            /**
             * Sets the used iterative closest point algorithm during the 
             * multiple registration of the point clouds. A default 
             * ICP algorithm is set at the instantiation of the 
             * fpcfMultipleRegistration object and can be changed by using this
             * method.
             *
             * @param[in] fpcfIcpPairAlignment the ICP algorithm for the fine 
             *                                alignment of two point clouds
             */
            void setICPPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment);

            /**
             * Sets the used global alignment algorithm during the multiple 
             * registration of the point clouds. A default global alignment
             * algorithm is set at the instantiation of the 
             * fpcfMultipleRegistration object and can be changed by using this
             * method.
             *
             * @param[in] fpcfLuMGlobalAlignment the global alignment algorithm 
             *                                  to reduce the accumulated error
             *                                  of the successive pairwise 
             *                                  alignment
             */
            void setLuMGlobalAlignment(typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment);

            /**
             * Getter for the currently used fpcfPointClouds of the multiple 
             * registration
             *
             * @return returns the vector of pointClouds
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> getPointClouds();

            /**
             * Getter for the currently used point cloud preparator to prepare
             * the input point cloud.
             *
             * @return returns the current point cloud preparator
             */
            typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr getPreparator();

            /**
             * Getter for the currently used correspondence rejector to reject 
             * wrong corresponding point pairs the input point cloud.
             *
             * @return returns the current correspondence rejector
             */
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr getCorrespondenceRejector();

            /**
             * Getter for the currently used iteratice closest point algorithm
             * to fine align the input point clouds.
             *
             * @return returns the current ICP algorithm object
             */
            typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr getICPPairAlignment();

            /**
             * Getter for the currently used global alignment algorithm to 
             * reduce the accumulated error of the successive pairwise 
             * alignment
             *
             * @return returns the current ICP algorithm object
             */
            typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr getLuMGlobalAlignment();

            /**
             * Getter for the resulting transformations to align all input 
             * point clouds. The resulting map contains the id of the point 
             * cloud and the according transformation matrix. The map contains
             * only meaningful values, if performRegistration() was invoked beforehand.
             *
             * @returns a map containing the point cloud id and the according 
             *          transformation
             */
            std::map<std::string, Eigen::Matrix4f> getTransformationMap();

            /**
             * Invokes the complete registration of the input point clouds. The
             * positions of the input point clouds will be directly changed! 
             * The resulting transformation map can be accessed using the 
             * getTransformationMap() method.
             */
            void performRegistration();

        protected:
            
            
        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds_;
            typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator_;
            boost::shared_ptr<fpcf::fpcfGraphOptimization<PointT, DescriptorT>> graphOptimizer;
            typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphTypePtr minSpanTree_;
            std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> predecessorMap_;
            std::map<std::string, Eigen::Matrix4f> transformationMap_;
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector_;
            typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment_;
            typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment_;

            fpcfMultipleRegistration(); // hide default constructor
            void initTransformationMap();
            void preparePointClouds();
            void graphOptimization();
            void createRegistrationPairs();
            void alignPointCloudPairs();
            void TransformationMapAccumulation();
            void alignPointCloudsGlobal();
            void globalLuM();
            void init();
            
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::fpcfMultipleRegistration(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator)
    {
        setPointCloudPreparator(fpcfPointCloudPreparator);
        init();
    }
            
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::~fpcfMultipleRegistration()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        pointClouds_.push_back(fpcfPointCloud);
    }
     
    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::addPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        for (int pcNr = 0; pcNr < fpcfPointClouds.size(); pcNr++)
        {
            fpcfPointClouds_.push_back(fpcfPointClouds.at(pcNr));
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        pointClouds_ = fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::setPointCloudPreparator(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPointCloudPreparator)
    {
        fpcfPointCloudPreparator_ = fpcfPointCloudPreparator;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector)
    {
        fpcfCorrespondenceRejector_ = fpcfCorrespondenceRejector;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::setICPPairAlignment(typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignment)
    {
        fpcfIcpPairAlignment_ = fpcfIcpPairAlignment;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::setLuMGlobalAlignment(typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr fpcfLuMGlobalAlignment)
    {
        fpcfLuMGlobalAlignment_ = fpcfLuMGlobalAlignment;
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr>
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getPointClouds()
    {
        return pointClouds_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getPreparator()
    {
        return fpcfPreparator_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getCorrespondenceRejector()
    {
        return fpcfCorrespondenceRejector_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getICPPairAlignment()
    {
        return fpcfICPPairAlignment_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>::Ptr
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getLuMGlobalAlignment()
    {
        return fpcfLuMGlobalAlignment_;
    }

    template <typename PointT, typename DescriptorT>
    std::map<std::string, Eigen::Matrix4f>
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::getTransformationMap()
    {
        return transformationMap_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::performRegistration()
    {
        // start clock
        clock_t start, end;
        start = clock();

        initTransformationMap();
        preparePointClouds();
        graphOptimization();
        createRegistrationPairs();
        alignPointCloudPairs();
        alignPointCloudsGlobal();
        if (boost::num_vertices(*minSpanTree_) > 2)
        {
            globalLuM();
            TransformationMapAccumulation();
        }

        // stop clock
        end = clock();
        cout << endl << "    total computation time: " << (float)(end - start) / CLOCKS_PER_SEC << " s" << endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::initTransformationMap()
    {
        transformationMap_.clear();
        for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
        {
            transformationMap_.insert(std::pair<std::string, Eigen::Matrix4f>(pointClouds_.at(pcNr)->getId(), Eigen::Matrix4f::Identity()));
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::preparePointClouds()
    {
        // start clock
        clock_t start, end;
        start = clock();

        fpcfPointCloudPreparator_->setPointClouds(pointClouds_);
        fpcfPointCloudPreparator_->prepare();

        // stop clock
        end = clock();
        cout << endl << "    preparation computation time: " << (float)(end - start) / CLOCKS_PER_SEC << " s" << endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::graphOptimization()
    {
        // start clock
        clock_t start, end;
        start = clock();

        fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejectorCopy(new fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>(*fpcfCorrespondenceRejector_));
        graphOptimizer.reset(new fpcf::fpcfGraphOptimization<PointT, DescriptorT>(pointClouds_));
        graphOptimizer->setCorrespondenceRejector(fpcfCorrespondenceRejectorCopy);
        graphOptimizer->performOptimization();
        predecessorMap_ = graphOptimizer->getPredecessorMap();
        minSpanTree_ = graphOptimizer->getMinimumSpanningTree();

        // stop clock
        end = clock();
        cout << endl << "    MST computation time: " << (float)(end - start) / CLOCKS_PER_SEC << " s" << endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::createRegistrationPairs()
    {
        // create pairs
        for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pcNr = 0; pcNr < boost::num_vertices(*minSpanTree_); pcNr++)
        {
            fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor predecessor = predecessorMap_[pcNr];
            if (predecessor != pcNr)
            {
                fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair(new fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>());
                fpcfPointCloudPair->setPointCloudSource((*minSpanTree_)[pcNr]);
                fpcfPointCloudPair->setPointCloudTarget((*minSpanTree_)[predecessor]);
                // reuse initial information from minSpanTree computation
                Eigen::Matrix4f transformation = (*minSpanTree_).get_edge(predecessor, pcNr).second.fpcfPointCloudPair->getTransformation();
                if (pcNr < predecessor)
                {
                    transformation = transformation.inverse().eval();
                }
                fpcfPointCloudPair->setTransformation(transformation);
                (*minSpanTree_).get_edge(predecessor, pcNr).second.edgeWeight = 1.0;
                (*minSpanTree_).get_edge(predecessor, pcNr).second.fpcfPointCloudPair = fpcfPointCloudPair;
            }
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::alignPointCloudPairs()
    {
        // start clock
        clock_t start, end;
        start = clock();

        // parallelized
        boost::thread_group thrdGrp;
        std::vector<fpcf::fpcfPairRegistration<PointT, DescriptorT>*> fpcfRegistrationPairs;
        for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pcNr = 0; pcNr < boost::num_vertices(*minSpanTree_); pcNr++)
        {
            fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pred = predecessorMap_[pcNr];
            if (pred != pcNr)
            {
                fpcf::fpcfPairRegistration<PointT, DescriptorT> *fpcfPairRegistration = new fpcf::fpcfPairRegistration<PointT, DescriptorT>();
                fpcfRegistrationPairs.push_back(fpcfPairRegistration);
                fpcf::fpcfICPPairAlignment<PointT, DescriptorT>::Ptr fpcfIcpPairAlignmentCopy(new fpcf::fpcfICPPairAlignment<PointT, DescriptorT>(*fpcfIcpPairAlignment_));
                fpcfPairRegistration->setIcpPairAlignment(fpcfIcpPairAlignmentCopy);
                fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair = (*minSpanTree_).get_edge(pred, pcNr).second.fpcfPointCloudPair;
                fpcfPairRegistration->setPointCloudPair(fpcfPointCloudPair);
                thrdGrp.create_thread(boost::bind(&fpcf::fpcfPairRegistration<PointT, DescriptorT>::performRegistration, fpcfPairRegistration));
            }
        }
        thrdGrp.join_all();
        for (int regPairNr = 0; regPairNr < fpcfRegistrationPairs.size(); regPairNr++)
        {
            delete fpcfRegistrationPairs.at(regPairNr);
        }

        // stop clock
        end = clock();
        cout << endl << "    pairwise registration computation time: " << (float)(end - start) / CLOCKS_PER_SEC << " s" << endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::alignPointCloudsGlobal()
    {
        Eigen::Matrix4f accumulatedTrans;
        for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pcNr = 0; pcNr < boost::num_vertices(*minSpanTree_); pcNr++)
        {
            int curr = pcNr;
            fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pred = predecessorMap_[curr];
            if (pred != curr)
            {
                fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair = (*minSpanTree_).get_edge(pred, pcNr).second.fpcfPointCloudPair;
                fpcf::fpcfPointCloudTransformation<PointT, DescriptorT> fpcfPointCloudTransformation(fpcfPointCloudPair->getPointCloudSource());
                accumulatedTrans = accumulatedTrans.Identity();
                while (pred != curr)
                {
                    fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair = (*minSpanTree_).get_edge(pred, curr).second.fpcfPointCloudPair;
                    accumulatedTrans = fpcfPointCloudPair->getTransformation() * accumulatedTrans;
                    curr = pred;
                    pred = predecessorMap_[curr];
                }
                fpcfPointCloudTransformation.setTransformation(accumulatedTrans);
                transformationMap_.erase(fpcfPointCloudPair->getPointCloudSource()->getId());
                transformationMap_.insert(std::pair<std::string, Eigen::Matrix4f>(fpcfPointCloudPair->getPointCloudSource()->getId(), accumulatedTrans));
                fpcfPointCloudTransformation.transformPointCloud();
                fpcfPointCloudTransformation.transformMostDownsampledPointCloud();
            }
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::globalLuM()
    {
        // start clock
        clock_t start, end;
        start = clock();

        std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds;
        for (fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor pcNr = 0; pcNr < boost::num_vertices(*minSpanTree_); pcNr++)
        {
            pointClouds.push_back((*minSpanTree_)[pcNr]);
        }
        fpcfLuMGlobalAlignment_->setPointClouds(pointClouds);
        fpcfLuMGlobalAlignment_->setPredecessorMap(predecessorMap_);
        fpcfLuMGlobalAlignment_->performAlignment();

        // stop clock
        end = clock();
        cout << endl << "    LuM computation time: " << (float)(end - start) / CLOCKS_PER_SEC << " s" << endl;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::TransformationMapAccumulation()
    {
        std::map<std::string, Eigen::Matrix4f> gICPTransformationMap = fpcfLuMGlobalAlignment_->getTransformationMap();
        for (int pcNr = 0; pcNr < pointClouds_.size(); pcNr++)
        {
            Eigen::Matrix4f graphTrans = transformationMap_.find(pointClouds_.at(pcNr)->getId())->second;
            Eigen::Matrix4f gICPTrans = gICPTransformationMap.find(pointClouds_.at(pcNr)->getId())->second;
            transformationMap_.erase(pointClouds_.at(pcNr)->getId());
            transformationMap_.insert(std::pair<std::string, Eigen::Matrix4f>(pointClouds_.at(pcNr)->getId(), gICPTrans * graphTrans));
        }
    }

    template <typename PointT, typename DescriptorT>
    void 
    fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::init()
    {
        fpcfCorrespondenceRejector_.reset(new fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>());
        fpcfIcpPairAlignment_.reset(new fpcf::fpcfICPPairAlignment<PointT, DescriptorT>());
        fpcfLuMGlobalAlignment_.reset(new fpcf::fpcfLuMGlobalAlignment<PointT, DescriptorT>());
    }

} // end namespace fpcf

#endif // #ifndef _FPCFMULTIPLEREGISTRATION_H_