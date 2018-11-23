/**
 * @file    multipleCloudFusion.cpp
 * @author  Thomas Weber
 *
 * @description This is an example of the point cloud fusion framework to fuse
 *              multiple point clouds.
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

// FPCF
#include <fpcf/io/fpcfPointCloudReader.h>
#include <fpcf/io/fpcfPointCloudWriter.h>
#include <fpcf/registration/fpcfMultipleRegistration.h>
#include <fpcf/filter/fpcfVoxelDownsampling.h>
#include <fpcf/filter/fpcfDistanceFilter.h>
#include <fpcf/filter/fpcfPlaneFilter.h>
#include <fpcf/filter/fpcfNoiseFilter.h>
#include <fpcf/registration/features/detectors/fpcfNarfKeypoint.h>
#include <fpcf/registration/features/detectors/fpcfSiftKeypoint.h>
#include <fpcf/registration/features/descriptors/fpcfFPFHDescriptor.h>
#include <fpcf/tools/fpcfVisualizer.h>

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::FPFHSignature33 DescriptorType;

vector<fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr> pointClouds;

// parameters
const float FILTER_DISTANCE = 1.5;
const float VOXEL_LEAF_SIZE = 0.01f;
const int MAX_PLANE_SIZE = 1000;
const float NOISE_OUTLINER_RADIUS = 0.02f;
const float DESCRIPTOR_RADIUS = 0.035f;

int parseArgs (int argc, char **argv)
{
    int loadResult = 1;
    for (int argNr = 1; argNr < argc; argNr++)
    {
        char idBuf [32];
        int idBase = 10;
        _itoa(argNr, idBuf, idBase);
        fpcf::fpcfPointCloudData<PointType, DescriptorType>::Ptr newPointCloud(new fpcf::fpcfPointCloudData<PointType, DescriptorType>(idBuf));
        std::string pcdFileName = std::string(argv[argNr]);
        fpcf::fpcfPointCloudReader<PointType, DescriptorType> *reader = new fpcf::fpcfPointCloudReader<PointType, DescriptorType>();
        loadResult = reader->loadPCDFile("", pcdFileName, newPointCloud) == 0 && loadResult;
        pointClouds.push_back(newPointCloud);
    }
    return loadResult;
}

int main (int argc, char** argv)
{
    // load point cloud data
    if (!parseArgs(argc, argv))
    {
        PCL_ERROR("Usage: \n");
        PCL_ERROR("    multipleCloudFusion.exe PCDFILE1 FPCFFILE2 [FPCFFILE_N]* \n");
        PCL_ERROR("Example: \n");
        PCL_ERROR("    multipleCloudFusion.exe points0000.pcd points0005.pcd");
        return -1;
    }
    
    // create point cloud preparator
    fpcf::fpcfPointCloudPreparation<PointType, DescriptorType>::Ptr fpcfPointCloudPreparator(new fpcf::fpcfPointCloudPreparation<PointType, DescriptorType>());

    // uncomment to use the persistent features computation:
        //fpcfPointCloudPreparator->calculatePersistentFeature(true);

    // add filters
    fpcf::fpcfVoxelDownsampling<PointType, DescriptorType>::Ptr fpcfDownsampler(new fpcf::fpcfVoxelDownsampling<PointType, DescriptorType>(VOXEL_LEAF_SIZE));
    fpcf::fpcfDistanceFilter<PointType, DescriptorType>::Ptr fpcfDistanceFilter(new fpcf::fpcfDistanceFilter<PointType, DescriptorType>(fpcf::Direction::Z, FILTER_DISTANCE));
    fpcf::fpcfPlaneFilter<PointType, DescriptorType>::Ptr fpcfPlaneFilter(new fpcf::fpcfPlaneFilter<PointType, DescriptorType>(MAX_PLANE_SIZE));
    fpcf::fpcfNoiseFilter<PointType, DescriptorType>::Ptr fpcfNoiseFilter(new fpcf::fpcfNoiseFilter<PointType, DescriptorType>(NOISE_OUTLINER_RADIUS));

    fpcfPointCloudPreparator->addFilter(fpcfDownsampler);
    fpcfPointCloudPreparator->addFilter(fpcfDistanceFilter);
    fpcfPointCloudPreparator->addFilter(fpcfPlaneFilter);
    fpcfPointCloudPreparator->addFilter(fpcfNoiseFilter);

    // set keypoint detector
    // uncomment to use the NARF keypoint detector:
        //fpcf::fpcfNarfKeypoint<PointType, DescriptorType>::Ptr fpcfNarfKeypointDetector(new fpcf::fpcfNarfKeypoint<PointType, DescriptorType>());
        //fpcfPointCloudPreparator->setDetector(fpcfNarfKeypointDetector);
    // uncomment to use the SIFT keypoint detector:
        //fpcf::fpcfSiftKeypoint<PointType, DescriptorType>::Ptr fpcfSiftKeypointDetector(new fpcf::fpcfSiftKeypoint<PointType, DescriptorType>());
        //fpcfPointCloudPreparator->setDetector(fpcfSiftKeypointDetector);

    // set point descriptor
    fpcf::fpcfFPFHDescriptor<PointType>::Ptr fpcfFPFHDescriptor(new fpcf::fpcfFPFHDescriptor<PointType>(DESCRIPTOR_RADIUS));
    fpcfPointCloudPreparator->setDescriptor(fpcfFPFHDescriptor);

    // create point cloud graph
    fpcf::fpcfMultipleRegistration<PointType, DescriptorType> globalRegistration(fpcfPointCloudPreparator);
    globalRegistration.setPointClouds(pointClouds);
    globalRegistration.performRegistration();

    // write results
    fpcf::fpcfPointCloudWriter<PointType, DescriptorType> writer;
    writer.writeMergedPointClouds(pointClouds, "mergedPointClouds");
    writer.writeMergedDownsampledPointClouds(pointClouds, "mergedDownsampledPointClouds");

    // visualize results
    fpcf::fpcfVisualizer<PointType, DescriptorType> *vis = new fpcf::fpcfVisualizer<PointType, DescriptorType>();
    vis->setPointCloudColor(fpcf::COLOR_STATE::NORMAL);
    for (int pcNr = 0; pcNr < pointClouds.size(); pcNr++)
    {
        vis->addPointCloud(pointClouds.at(pcNr));
    }
    vis->visualize();

    cout << "Finished!" << endl;
}