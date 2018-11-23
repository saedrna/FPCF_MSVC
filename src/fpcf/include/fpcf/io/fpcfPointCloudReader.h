/**
 * @file    fpcfPointCloudReader.h
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

#ifndef _FPCFPOINTCLOUDREADER_H_
#define _FPCFPOINTCLOUDREADER_H_

// STL
#include <string>

// Boost

// PCL
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{

    /**
     * This class reads *.pcd point cloud files into a fpcfPointCloud.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudReader
    {
        public:

            /**
             * Constructor that creates a new fpcfPointCloudReader.
             */
            fpcfPointCloudReader();

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudReader();

            /**
             * Reads a point cloud from a pcd file in.
             * 
             * @param[in]  path       path to the pcd file
             * @param[in]  fileName   name of the pcd file
             * @param[out] pointCloud the resulting fpcfPointCloud object
             * @return returns -1 on error and 0 on success
             */
            int loadPCDFile(std::string path, std::string fileName, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud);

        protected:

        private:
            void removeNAN(typename pcl::PointCloud<PointT>::Ptr pointCloud);
    };

    template <typename PointT, typename DescriptorT>
    fpcfPointCloudReader<PointT, DescriptorT>::fpcfPointCloudReader()
    {
    
    }
    
    template <typename PointT, typename DescriptorT>
    fpcfPointCloudReader<PointT, DescriptorT>::~fpcfPointCloudReader()
    {

    }

    template <typename PointT, typename DescriptorT>
    int
    fpcfPointCloudReader<PointT, DescriptorT>::loadPCDFile(std::string path, std::string fileName, typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud)
    {
        pcl::PointCloud<PointT>::Ptr newCloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(path + fileName, *newCloud) == -1)
        {
            PCL_ERROR("Couldn't read file /n");
            return -1;
        }
        std::cout << "Loaded " << newCloud->width * newCloud->height << " data points from " << fileName << std::endl;
        removeNAN(newCloud);
        pointCloud->setPointCloud(newCloud);
        return 0;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcfPointCloudReader<PointT, DescriptorT>::removeNAN(typename pcl::PointCloud<PointT>::Ptr pointCloud)
    {
        std::vector<int> removedIndices;
        pcl::removeNaNFromPointCloud(*pointCloud, *pointCloud, removedIndices);
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDREADER_H_