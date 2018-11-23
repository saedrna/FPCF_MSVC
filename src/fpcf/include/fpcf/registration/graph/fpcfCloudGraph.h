/**
 * @file    fpcfCloudGraph.h
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

#ifndef _FPCFCLOUDGRAPH_H_
#define _FPCFCLOUDGRAPH_H_

// STL

// Boost
#include <boost/graph/adjacency_matrix.hpp>

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/data/fpcfPointCloudDataPair.h>

namespace fpcf
{
    /**
     * This class represents the graph, which is used during the global point 
     * graph computation. The nodes represent the fpcfPointClouds and the edges 
     * contain a weight and the fpcfPointCloudPair containing the fpcfPointCloud 
     * of the connected nodes.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfCloudGraph
    {
        public:
            /**
             * Struct for the edges, which contain a weight and a 
             * fpcfPointCloudPair for the connected nodes
             */
            struct EdgeType
            {
                double edgeWeight;
                typename fpcf::fpcfPointCloudDataPair<PointT, DescriptorT>::Ptr fpcfPointCloudPair;
            };

            /**
             * Describes the graph as an ajacency matrix with undirected edges.
             */
            typedef boost::adjacency_matrix<
                boost::undirectedS,
                typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr,
                EdgeType> GraphType;

            typedef boost::shared_ptr<GraphType> GraphTypePtr;

        protected:

        private:
            /**
             * hide default constructor
             */
            fpcfCloudGraph();

            /**
             * hide destructor
             */
            virtual ~fpcfCloudGraph();
    };

} // end namespace fpcf

#endif // #ifndef _FPCFCLOUDGRAPH_H_