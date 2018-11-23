/**
 * @file    fpcfGuiMultipleFusion.h
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

#ifndef _FPCFGUIMULTIPLEFUSION_H_
#define _FPCFGUIMULTIPLEFUSION_H_

// STL
#include <fstream>

// Boost

// PCL

// FPCF
#include <fpcf/registration/fpcfMultipleRegistration.h>

namespace fpcf_gui
{
    /**
     * This class is a specialization of the fpcfMultipleRegistration class for 
     * the fpcfGUI.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfGuiMultipleFusion : public fpcf::fpcfMultipleRegistration<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfGuiMultipleFusion<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfGuiMultipleFusion with a 
             * specified point cloud preparator.
             *
             * @param[in] fpcfPreparator the point cloud preparator to prepare 
             *                          the input point clouds before the 
             *                          fusion process
             */
            fpcfGuiMultipleFusion(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPreparator);

            /**
             * Destructor
             */
            virtual ~fpcfGuiMultipleFusion();

            /**
             * Performs the multiple fusion process of the input point clouds 
             * and prints out the computed transformation matrices into a text 
             * file.
             */
            void performRegistration();

            /**
             * Transforms the input point clouds using the computed 
             * transformations of the last performFusion() call. The ids of the
             * input point clouds needs to be the same as during the fusion 
             * process.
             */
            void transformPointClouds();

        protected:

        private:
            void writeTransformationMatrices();

    };

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::fpcfGuiMultipleFusion(typename fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::Ptr fpcfPreparator)
        : fpcfMultipleRegistration(fpcfPreparator)
    {

    }

    template <typename PointT, typename DescriptorT>
    fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::~fpcfGuiMultipleFusion()
    {

    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::performRegistration()
    {
        if (getPointClouds().size() > 0)
        {
            fpcf::fpcfMultipleRegistration<PointT, DescriptorT>::performRegistration();        
            writeTransformationMatrices();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::transformPointClouds()
    {
        std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds = getPointClouds();
        std::map<std::string, Eigen::Matrix4f> transformationMap = getTransformationMap();
        for(int pcNr = 0; pcNr < pointClouds.size(); pcNr++)
        {
            fpcf::fpcfPointCloudTransformation<PointT, DescriptorT> fpcfPointCloudTransformation(pointClouds.at(pcNr));
            fpcfPointCloudTransformation.setTransformation(transformationMap.find(pointClouds.at(pcNr)->getId())->second);
            fpcfPointCloudTransformation.transformPointCloud();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf_gui::fpcfGuiMultipleFusion<PointT, DescriptorT>::writeTransformationMatrices()
    {
        ofstream myfile;
        myfile.open("transformations.txt");
        if (myfile.is_open())
        {
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> pointClouds = getPointClouds();
            std::map<std::string, Eigen::Matrix4f> transformationMap = getTransformationMap();
            for(int pcNr = 0; pcNr < pointClouds.size(); pcNr++)
            {
                Eigen::Matrix4f transformation = transformationMap.find(pointClouds.at(pcNr)->getId())->second;
                myfile << pointClouds.at(pcNr)->getId() << " = " << "[ " << std::endl;
                myfile << transformation(0) << ", "; 
                myfile << transformation(1) << ", ";
                myfile << transformation(2) << ", ";
                myfile << transformation(3) << "; " << std::endl;
                myfile << transformation(4) << ", ";
                myfile << transformation(5) << ", ";
                myfile << transformation(6) << ", ";
                myfile << transformation(7) << "; " << std::endl;
                myfile << transformation(8) << ", ";
                myfile << transformation(9) << ", ";
                myfile << transformation(10) << ", ";
                myfile << transformation(11) << "; " << std::endl;
                myfile << transformation(12) << ", ";
                myfile << transformation(13) << ", ";
                myfile << transformation(14) << ", ";
                myfile << transformation(15) << "];" << std::endl;
            }
            myfile.close();
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFGUIMULTIPLEFUSION_H_