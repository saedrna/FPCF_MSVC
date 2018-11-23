/**
 * @file    fpcfMultipleKinectGrabber.h
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

#ifndef _FPCFMULTIPLEKINECTGRABBER_H_
#define _FPCFMULTIPLEKINECTGRABBER_H_

// STL
#include <string>

// Boost
#include <boost/thread.hpp>

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/io/fpcfKinectGrabber.h>

namespace fpcf
{

    /**
     * This class synchronizes multiple Kinect sensors and grabs the output 
     * simultaneously.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfMultipleKinectGrabber
    {
        
        public:
            /**
             * Constructor that creates a new empty fpcfMultipleKinectGrabber
             */
            fpcfMultipleKinectGrabber();

            /**
             * Destructor
             */
            virtual ~fpcfMultipleKinectGrabber();

            /**
             * Adds a new Kinect sensor with a specified id to the multiple 
             * Kinect grabber. Before adding a new Kinect device, the multiple 
             * Kinect Grabber should be stop before.
             *
             * @param[in] kinectDeviceId the id of the Kinect sensor. I.e. "#1"
             *                           for the first Kinect sensor.
             */
            void addKinectDevice(std::string kinectDeviceId);

            /**
             * Removes a new Kinect sensor with a specified id from the 
             * multiple Kinect grabber. Before removing a new Kinect device,
             * the multiple Kinect Grabber should be stop beforehand.
             *
             * @param[in] kinectDeviceId the id of the Kinect sensor. I.e. "#1"
             *                           for the first Kinect sensor.
             */
            void removeKinectDevice(std::string kinectDeviceId);

            /**
             * Stops and removes all Kinects sensors from the multiple Kinect 
             * Grabber.
             */
            void removeAllKinectDevices();

            /**
             * Starts the grabbing process of all added Kinect sensors.
             */
            void start();

            /**
             * Stops the grabbing process of all added Kinect sensors.
             */
            void stop();

            /**
             * Grabs new point clouds from all Kinect sensors. The multiple
             * Kinect grabber needs to be started beforehand.
             *
             * @return returns new grabbed point clouds of all Kinect sensors
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> getGrabbedPointClouds();
        
        protected:
            
        private:
            std::vector<fpcf::fpcfKinectGrabber<PointT, DescriptorT>*> kinectGrabbers_;

            void startKinectGrabbers();
            void stopKinectGrabbers();
            void setGrabbedPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr *fpcfPointCloud, fpcf::fpcfKinectGrabber<PointT, DescriptorT>* grabber);
    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::fpcfMultipleKinectGrabber()
    {

    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::~fpcfMultipleKinectGrabber()
    {
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
            delete kinectGrabbers_.at(i);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::addKinectDevice(std::string kinectDeviceId)
    {
        try
        {
            fpcf::fpcfKinectGrabber<PointT, DescriptorT> *newKinectGrabber = new fpcf::fpcfKinectGrabber<PointT, DescriptorT>(kinectDeviceId);
            kinectGrabbers_.push_back(newKinectGrabber);
        }
        catch (...)
        {
            std::cout << "Could not create Kinect grabber with device id: " << kinectDeviceId << std::endl;
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::removeKinectDevice(std::string kinectDeviceId)
    {
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
            if (kinectGrabbers_.at(i)->getKinectDeviceId() == kinectDeviceId)
            {
                kinectGrabbers_.at(i)->stop();
                delete kinectGrabbers_.at(i);
                kinectDeviceIds_.erase(i);
            }
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::removeAllKinectDevices()
    {
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
                kinectGrabbers_.at(i)->stop();
                delete kinectGrabbers_.at(i);
        }
        kinectGrabbers_.clear();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::start()
    {
        startKinectGrabbers();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::stop()
    {
        stopKinectGrabbers();
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr>
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::getGrabbedPointClouds()
    {
        boost::thread_group thrdGrp;
        std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> grabbedPointClouds(kinectGrabbers_.size());
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
            setGrabbedPointCloud(&grabbedPointClouds.at(i), kinectGrabbers_.at(i));
        }
        thrdGrp.join_all();
        return grabbedPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::startKinectGrabbers()
    {
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
            kinectGrabbers_.at(i)->start();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::stopKinectGrabbers()
    {
        for (int i = 0; i < kinectGrabbers_.size(); i++)
        {
            kinectGrabbers_.at(i)->stop();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfMultipleKinectGrabber<PointT, DescriptorT>::setGrabbedPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr *fpcfPointCloud, fpcf::fpcfKinectGrabber<PointT, DescriptorT>* grabber)
    {
        *fpcfPointCloud = grabber->getGrabbedPointCloud();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFMULTIPLEKINECTGRABBER_H_