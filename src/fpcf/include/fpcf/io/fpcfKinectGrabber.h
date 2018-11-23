/**
 * @file    fpcfKinectGrabber.h
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

#ifndef _FPCFKINECTGRABBER_H_
#define _FPCFKINECTGRABBER_H_

// STL
#include <string>

// Boost

// PCL
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>

namespace fpcf
{

    /**
     * This class grabs the output from a single Kinect device and creates a 
     * fpcfPointCloud from the output stream.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfKinectGrabber
    {
        
        public:

            /**
             * Constructor that creates a new fpcfKinectGrabber for a Kinect 
             * device with the specified id.
             *
             * @param[in] kinectDeviceId the id of the Kinect device. I.e. "#1"
             *                           for the first Kinect sensor.
             */
            fpcfKinectGrabber(std::string kinectDeviceId);

            /**
             * Destructor
             */
            virtual ~fpcfKinectGrabber();

            /**
             * Getter for the id of the used Kinect sensor.
             *
             * @return the id of the setted Kinect sensor 
             */
            std::string getKinectDeviceId();

            /**
             * Starts the Kinect sensor to grab the point clouds.
             */
            void start();

            /**
             * Stops the grabbing of point clouds by the Kinect sensor.
             */
            void stop();

            /**
             * Returns the next grabbed point cloud of the Kinect sensors as 
             * fpcfPointCloud. The Kinect grabber need to be started before.
             *
             * @return returns a new grabbed point cloud
             */
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr getGrabbedPointCloud();
        
        protected:
            
        private:
            std::string kinectDeviceId_;
            pcl::Grabber* kinectGrabber_;
            typename pcl::PointCloud<PointT>::Ptr pclPointCloud_;
            typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfGrabbedPointCloud_;
            bool newCloud;
            boost::mutex cloudMutex;
            boost::condition_variable_any newCond;

            /**
             * hide default constructor
             */
            fpcfKinectGrabber();

            void cloudGrabCallback(const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::fpcfKinectGrabber(std::string kinectDeviceId)
    {
        newCloud = false;
        kinectDeviceId_ = kinectDeviceId;
        kinectGrabber_ = new pcl::OpenNIGrabber(kinectDeviceId_);
        boost::function<void (const pcl::PointCloud<PointType>::ConstPtr&)> callback = boost::bind(&fpcfKinectGrabber::cloudGrabCallback, this, _1);
        kinectGrabber_->registerCallback(callback);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::~fpcfKinectGrabber()
    {
        delete kinectGrabber_;
    }

    template <typename PointT, typename DescriptorT>
    std::string 
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::getKinectDeviceId()
    {
        return kinectDeviceId_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::start()
    {
        kinectGrabber_->start();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::stop()
    {
        kinectGrabber_->stop();
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::getGrabbedPointCloud()
    {
        cloudMutex.lock();
        while(!newCloud)
        {
            newCond.wait(cloudMutex);
        }
        pcl::PointCloud<PointT>::Ptr pclPointCloudCopy(new pcl::PointCloud<PointT>(*pclPointCloud_));
        newCloud = false;
        cloudMutex.unlock();
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfGrabbedPointCloud_(new fpcfPointCloudData<PointT, DescriptorT>(pclPointCloudCopy, getKinectDeviceId()));
        return fpcfGrabbedPointCloud_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfKinectGrabber<PointT, DescriptorT>::cloudGrabCallback(const typename pcl::PointCloud<PointT>::ConstPtr &pclPointCloud)
    {
        cloudMutex.lock();
        pclPointCloud_ = pclPointCloud->makeShared();
        newCloud = true;
        cloudMutex.unlock();
        newCond.notify_all();
    }

} // end namespace fpcf

#endif // #ifndef _FPCFKINECTGRABBER_H_