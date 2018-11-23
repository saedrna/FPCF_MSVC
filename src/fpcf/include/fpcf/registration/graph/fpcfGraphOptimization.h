/**
 * @file    fpcfGraphOptimization.h
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

#ifndef _FPCFGRAPHOPTIMIZATION_H_
#define _FPCFGRAPHOPTIMIZATION_H_

// STL

// Boost
#include <boost/thread.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp>

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/data/fpcfPointCloudDataPair.h>
#include <fpcf/registration/graph/fpcfCloudGraph.h>

namespace fpcf
{

    /**
     * This class optimizes the point cloud pairs for the registration using a 
     * graph. It tries to find the best point cloud pairs, which probably share
     * the most overlapping area. Thereby a heuristic is used. The more 
     * correspondences between a point cloud pair are found, the higher the 
     * possibility that they share a more overlapping view. The input point 
     * clouds of this class need to be already prepared.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfGraphOptimization
    {

        public:

            /**
             * Constructor that creates a new fpcfGraphOptimization for a vector
             * of fpcfPointClouds. Each point cloud needs to be prepared 
             * beforehand.
             *
             * @param[in] fpcfPointClouds the vector of already prepared point 
             *                           clouds, which pairs will be optimized
             */
            fpcfGraphOptimization(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Destructor
             */
            virtual ~fpcfGraphOptimization();

            /**
             * Sets the fpcfPointClouds, which point pairs will be optimized
             *
             * @param[in] fpcfPointClouds a vector of already prepared point 
             *                           clouds, which pairs will be optimized
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Setter for the used correspondence rejector of the graph 
             * optimization. There is already a default correspondence rejector
             * set at the instantiation, but can be changed using this method.
             *
             * @param[in] fpcfCorrespondenceRejector the used correspondence 
             *                                      rejector to reject 
             *                                      correspondences between 
             *                                      point cloud pairs
             */
            void setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector);

            /**
             * Getter for the currently used fpcfPointClouds of the graph 
             * optimization
             *
             * @return returns the vector of pointClouds
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> getPointClouds();

            /**
             * Getter for the computed minimum spanning tree of the input point
             * clouds. This is null, if performOptimization() wasn't invoked 
             * before. The minimum spanning tree is a graph which connects the 
             * best point cloud pairs.
             *
             * @return returns the computed minimum spanning tree of the input 
             *         point clouds
             */
            typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphTypePtr getMinimumSpanningTree();

            /**
             * Getter for the computed predecessor map of the input point 
             * cloud. This is null, if performOptimization() wasn't invoked 
             * before. The predecessor map contains the point cloud source and 
             * point cloud target pairs to allign all point clouds into the 
             * coordinate system of the first point cloud of the input vector.
             *
             * @return the computed predecessor map to align all point clouds
             */
            std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> getPredecessorMap();

            /**
             * Invokes the the computation of the optimal point cloud pairs. 
             * The input point clouds need to be already prepared. The 
             * resulting minimum spanning tree and predecessor map can be 
             * accessed using the according methods after the optimization.
             */
            void performOptimization();

        protected:

        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds_;
            typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphTypePtr fpcfCloudGraph_;
            typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphTypePtr minSpanTree_;
            std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor> predecessorMap_;
            typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector_;

            /**
             * hide default constructor
             */
            fpcfGraphOptimization();

            void createCloudGraph();
            void calculateEdgeWeights();
            void calculateMinimumSpanningTree();
            void calculatePredecessorMap();
            void createPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget);
            void calculateCorrespondences(typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType *fpcfCloudGraphEdge, typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector);
            
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::fpcfGraphOptimization(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        setPointClouds(fpcfPointClouds);
        fpcfCorrespondenceRejector_.reset(new fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>());
    }
            
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::~fpcfGraphOptimization()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        fpcfPointClouds_ = fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::setCorrespondenceRejector(typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector)
    {
        fpcfCorrespondenceRejector_ = fpcfCorrespondenceRejector;
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr>
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::getPointClouds()
    {
        return fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphTypePtr
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::getMinimumSpanningTree()
    {
        return minSpanTree_;
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor>
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::getPredecessorMap()
    {
        return predecessorMap_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::performOptimization()
    {
        createCloudGraph();
        calculateEdgeWeights();
        calculateMinimumSpanningTree();
        calculatePredecessorMap();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::createCloudGraph()
    {
        fpcfCloudGraph_.reset(new fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType(fpcfPointClouds_.size()));
        for (int i = 0; i < fpcfPointClouds_.size(); i++)
        {
            (*fpcfCloudGraph_)[i] = fpcfPointClouds_.at(i);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::calculateEdgeWeights()
    {
        // parallelized
        boost::thread_group thrdGrp;
        std::vector<fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr> fpcfPointCloudPairs;
        for (int i = 0; i < fpcfPointClouds_.size(); i++)
        {
            for (int j = 0; j < i; j++)
            {
                // calculate correspondences
                fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair(new fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>());
                fpcfPointCloudPairs.push_back(fpcfPointCloudPair);
                createPointCloudPair(fpcfPointCloudPair, fpcfPointClouds_.at(i), fpcfPointClouds_.at(j));
                boost::add_edge(i, j, *fpcfCloudGraph_);
                fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType *edge = &(fpcfCloudGraph_->get_edge(i, j).second);
                edge->fpcfPointCloudPair = fpcfPointCloudPair;
                fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejectorCopy(new fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>(*fpcfCorrespondenceRejector_));
                thrdGrp.create_thread(boost::bind(&fpcf::fpcfGraphOptimization<PointT, DescriptorT>::calculateCorrespondences, this, edge, fpcfCorrespondenceRejectorCopy));
            }
        }
        thrdGrp.join_all();
    }
    
    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::calculateMinimumSpanningTree()
    {
        fpcf::fpcfPointCloudPairTransformationEstimation<PointT, DescriptorT> transformationEstimation;
        // calculate minimum spanning tree edges
        std::vector<boost::graph_traits<fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType>::edge_descriptor> spanningTreeEdges;
        boost::kruskal_minimum_spanning_tree(*fpcfCloudGraph_, std::back_inserter(spanningTreeEdges), boost::weight_map(boost::get(&fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType::edgeWeight, *fpcfCloudGraph_)));
        // add point clouds as nodes to new minimum spanning tree graph
        minSpanTree_.reset(new fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType(boost::num_vertices(*fpcfCloudGraph_)));
        for (int i = 0; i < boost::num_vertices(*fpcfCloudGraph_); i++)
        {
             (*minSpanTree_)[i] = (*fpcfCloudGraph_)[i];
        }
        // add minimum spanning tree edges
        for (int edgeNr = 0; edgeNr < spanningTreeEdges.size(); edgeNr++)
        {
            Eigen::Matrix4f transformation;
            boost::graph_traits<fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType>::vertex_descriptor sourceVertex = boost::source(spanningTreeEdges.at(edgeNr), *minSpanTree_);
            boost::graph_traits<fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType>::vertex_descriptor targetVertex = boost::target(spanningTreeEdges.at(edgeNr), *minSpanTree_);
            boost::add_edge(sourceVertex, targetVertex, *minSpanTree_);
            fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType *graphEdge = &(fpcfCloudGraph_->get_edge(sourceVertex, targetVertex).second);
            typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair = graphEdge->fpcfPointCloudPair;
            fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType *minSpanTreeEdge = &(minSpanTree_->get_edge(sourceVertex, targetVertex).second);
            transformationEstimation.setPointCloudPair(fpcfPointCloudPair);
            transformationEstimation.estimateAlignment(transformation);
            minSpanTreeEdge->fpcfPointCloudPair = graphEdge->fpcfPointCloudPair;
        }
        boost::write_graphviz(std::cout, *minSpanTree_);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::calculatePredecessorMap()
    {
        predecessorMap_.resize(boost::num_vertices(*minSpanTree_));
        fpcf::fpcfCloudGraph<PointT, DescriptorT>::GraphType::vertex_descriptor source = *(boost::vertices(*minSpanTree_).first);
        predecessorMap_[source] = source;
        boost::breadth_first_search(*minSpanTree_, source, boost::visitor(boost::make_bfs_visitor(boost::record_predecessors(&predecessorMap_[0], boost::on_tree_edge()))));
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::createPointCloudPair(typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudSource, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloudTarget)
    {
        fpcfPointCloudPair->setPointCloudSource(pointCloudSource->getMostDownsampledPointCloud());
        fpcfPointCloudPair->setPointCloudTarget(pointCloudTarget->getMostDownsampledPointCloud());
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfGraphOptimization<PointT, DescriptorT>::calculateCorrespondences(typename fpcf::fpcfCloudGraph<PointT, DescriptorT>::EdgeType *fpcfCloudGraphEdge, typename fpcf::fpcfCorrespondenceRejector<PointT, DescriptorT>::Ptr fpcfCorrespondenceRejector)
    {
        int nrCorrespondences = -1;
        fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair = fpcfCloudGraphEdge->fpcfPointCloudPair;
        fpcf::fpcfCorrespondenceFinder<PointT, DescriptorT> corresFind(fpcfPointCloudPair);
        fpcfCorrespondenceRejector->setPointCloudPair(fpcfPointCloudPair);
        corresFind.calculateCorrepondences();
        nrCorrespondences = fpcfCorrespondenceRejector->rejectCorrespondences();
        fpcfCloudGraphEdge->edgeWeight = 1.0 / nrCorrespondences;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFGRAPHOPTIMIZATION_H_