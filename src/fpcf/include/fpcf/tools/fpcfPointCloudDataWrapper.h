/**
 * @file    fpcfPointCloudDataWrapper.h
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
#ifndef _FPCFPOINTCLOUDWRAPPER_H_
#define _FPCFPOINTCLOUDWRAPPER_H_

// STL

// Boost

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{
    /**
     * Wraps a single fpcf point cloud into an other fpcf point cloud object with
     * a different point descriptor type. Also the downsampled versions are 
     * wrapped. Computed point descriptors of the input point cloud cannot be 
     * transferred.
     * 
     * @param[in] fpcfPointCloudDataIn the input point cloud with its original 
     *                                point descriptor type
     * @param[out] fpcfPointCloudDataOut the point cloud wrapped into a point 
     *                                  cloud object with the outgoing point 
     *                                  descriptor type
     */
    template <typename PointT, typename DescriptorInT, typename DescriptorOutT>
    void wrapPointCloudData(typename fpcf::fpcfPointCloudData<PointT, DescriptorInT>::Ptr fpcfPointCloudDataIn, typename fpcf::fpcfPointCloudData<PointT, DescriptorOutT>::Ptr &fpcfPointCloudDataOut)
    {
        fpcfPointCloudDataOut.reset(new fpcf::fpcfPointCloudData<PointT, DescriptorOutT>(fpcfPointCloudDataIn->getId()));
        fpcfPointCloudDataOut->setPointCloud(fpcfPointCloudDataIn->getPointCloud());
        fpcfPointCloudDataOut->setKeypointCloud(fpcfPointCloudDataIn->getKeypointCloud());
        fpcfPointCloudDataOut->setNormalsCloud(fpcfPointCloudDataIn->getNormalsCloud());
        if (fpcfPointCloudDataIn->getDownsampledPointCloud())
        {
            typename fpcf::fpcfPointCloudData<PointT, DescriptorOutT>::Ptr downsampledPointCloud;
            wrapPointCloudData<PointT, DescriptorInT, DescriptorOutT>(fpcfPointCloudDataIn->getDownsampledPointCloud(), downsampledPointCloud);
            fpcfPointCloudDataOut->setDownsampledPointCloud(downsampledPointCloud);
        }
    }

    /**
     * Wraps a vector of fpcf point clouds into an other fpcf point cloud vector 
     * with a different point descriptor type. Also the downsampled versions 
     * are wrapped. Computed point descriptors of the input point clouds cannot
     * be transferred.
     * 
     * @param[in] fpcfPointCloudsDataIn the vector input point clouds with the 
     *                                 original point descriptor type
     * @param[out] fpcfPointCloudsDataOut the point clouds wrapped into a vector
     *                                   of point clouds object with the 
     *                                   outgoing point descriptor type
     */
    template <typename PointT, typename DescriptorInT, typename DescriptorOutT>
    void wrapPointCloudsData(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorInT>::Ptr> fpcfPointCloudsDataIn, std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorOutT>::Ptr> &fpcfPointCloudsDataOut)
    {
        fpcfPointCloudsDataOut.clear();
        for (int pcNr = 0; pcNr < fpcfPointCloudsDataIn.size(); pcNr++)
        {
            typename fpcf::fpcfPointCloudData<PointT, DescriptorOutT>::Ptr wrappedPointCloud;
            wrapPointCloudData<PointT, DescriptorInT, DescriptorOutT>(fpcfPointCloudsDataIn.at(pcNr), wrappedPointCloud); 
            fpcfPointCloudsDataOut.push_back(wrappedPointCloud);
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDWRAPPER_H_