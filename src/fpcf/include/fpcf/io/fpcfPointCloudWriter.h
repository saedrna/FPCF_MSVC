/**
 * @file    fpcfPointCloudWriter.h
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

#ifndef _FPCFPOINTCLOUDWRITER_H_
#define _FPCFPOINTCLOUDWRITER_H_

// STL
#include <string>

// Boost

// PCL
#include <pcl/io/pcd_io.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/tools/fpcfPointCloudConcatenator.h>

namespace fpcf
{

    /**
     * This class writes fpcfPointClouds into a *.pcd files
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudWriter
    {
        public:

            /**
             * Constructor that creates a new fpcfPointCloudWriter
             */
            fpcfPointCloudWriter();

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudWriter();

            /**
             * Writes the pclPointCloud of a fpcfPointCloud into a *.pcd file
             *
             * @param[in] fpcfPointCloud the fpcfPointCloud
             * @param[in] fileName the name of the file. I.e. "pointCloud.pcd".
             */
            void writePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, std::string fileName);

            /**
             * Writes the keypoint point cloud of a fpcfPointCloud into a *.pcd 
             * file
             *
             * @param[in] fpcfPointCloud the fpcfPointCloud
             * @param[in] fileName the name of the file. I.e. "pointCloud.pcd".
             */
            void writeKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, std::string fileName);

            /**
             * Merges a vector of fpcfPointClouds and writes the resulting 
             * pclPointCloud into a *.pcd file
             *
             * @param[in] fpcfPointClouds the vector containing the 
             *                           fpcfPointClouds
             * @param[in] fileName the name of the file. I.e. 
             *                     "mergedPointClouds.pcd".
             */
            void writeMergedPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName);

            /**
             * Merges the downsampled point clouds of a vector of 
             * fpcfPointClouds and writes the resulting pclPointCloud into 
             * a *.pcd file
             *
             * @param[in] fpcfPointClouds the vector containing the 
             *                           fpcfPointClouds
             * @param[in] fileName the name of the file. I.e. 
             *                     "mergedPointClouds.pcd".
             */
            void writeMergedDownsampledPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName);

            /**
             * Merges the keypoint point clouds of a vector of 
             * fpcfPointClouds and writes the resulting pclPointCloud into 
             * a *.pcd file
             *
             * @param[in] fpcfPointClouds the vector containing the 
             *                           fpcfPointClouds
             * @param[in] fileName the name of the file. I.e. 
             *                     "mergedKeypoints.pcd".
             */
            void writeMergedKeypointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName);

            /**
             * Merges the keypoint point clouds of the downsmpled point clouds
             * of a vector of fpcfPointClouds and writes the resulting 
             * pclPointCloud into a *.pcd file
             *
             * @param[in] fpcfPointClouds the vector containing the 
             *                           fpcfPointClouds
             * @param[in] fileName the name of the file. I.e. 
             *                     "mergedKeypoints.pcd".
             */
            void writeMergedDownsampledKeypointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName);

        protected:

        private:
            void writeCloud(pcl::PointCloud<PointT> &pclPointCloud, std::string fileName);
            
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::fpcfPointCloudWriter()
    {
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::~fpcfPointCloudWriter()
    {
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, std::string fileName)
    {
        writeCloud(*fpcfPointCloud->getPointCloud(), fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeKeypointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud, std::string fileName)
    {
        writeCloud(*fpcfPointCloud->getKeypointCloud(), fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeMergedPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName)
    {
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatPointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>("PointCloudsConcat" + fpcfPointClouds.at(0)->getId()));
        fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT> pcConcat;
        pcConcat.setPointClouds(fpcfPointClouds);
        pcConcat.concat(concatPointCloud);
        writePointCloud(concatPointCloud, fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeMergedDownsampledPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName)
    {
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatPointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>("DownsampledPointCloudsConcat" + fpcfPointClouds.at(0)->getId()));
        fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT> pcConcat;
        for (int pcNr = 0; pcNr < fpcfPointClouds.size(); pcNr++)
        {
            if (fpcfPointClouds.at(pcNr)->getDownsampledPointCloud())
            {
                pcConcat.addPointCloud(fpcfPointClouds.at(pcNr)->getDownsampledPointCloud());
            }
        }
        pcConcat.concat(concatPointCloud);
        writePointCloud(concatPointCloud, fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeMergedKeypointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName)
    {
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatKeypointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>("KeypointCloudsConcat" + fpcfPointClouds.at(0)->getId()));
        fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT> pcConcat;
        pcConcat.setPointClouds(fpcfPointClouds);
        pcConcat.concat(concatKeypointCloud);
        writeKeypointCloud(concatKeypointCloud, fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeMergedDownsampledKeypointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds, std::string fileName)
    {
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr concatPointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>("DownsampledKeypointCloudsConcat" + fpcfPointClouds.at(0)->getId()));
        fpcf::fpcfPointCloudConcatenator<PointT, DescriptorT> pcConcat;
        for (int pcNr = 0; pcNr < fpcfPointClouds.size(); pcNr++)
        {
            if (fpcfPointClouds.at(pcNr)->getDownsampledPointCloud())
            {
                pcConcat.addPointCloud(fpcfPointClouds.at(pcNr)->getDownsampledPointCloud());
            }
        }
        pcConcat.concat(concatPointCloud);
        writeKeypointCloud(concatPointCloud, fileName);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudWriter<PointT, DescriptorT>::writeCloud(pcl::PointCloud<PointT> &pclPointCloud, std::string fileName)
    {
        pcl::io::savePCDFileBinary(fileName + ".pcd", pclPointCloud);
        std::cout << "Saved " << pclPointCloud.points.size() << " data points to " << fileName << ".pcd" << std::endl;
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDWRITER_H_