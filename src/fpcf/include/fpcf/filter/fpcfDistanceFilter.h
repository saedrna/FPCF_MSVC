/**
 * @file    fpcfDistanceFilter.h
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

#ifndef _FPCFDISTANCEFILTER_H_
#define _FPCFDISTANCEFILTER_H_

// STL

// Boost

// PCL
#include <pcl/filters/passthrough.h>

// FPCF
#include <fpcf/filter/fpcfFilter.h>

namespace fpcf
{
    /**
     * Enumeration to define the direction of the distance filtering.
     */
    enum Direction { X, Y, Z };

    /**
     * This class filters points from fpcfPointCloudData data at a certain 
     * distance in a specidied direction from the origin of the coordinate 
     * system.
     */
    template <typename PointT, typename DescriptorT>
    class fpcfDistanceFilter : public fpcfFilter<PointT, DescriptorT>
    {
        public:
            typedef boost::shared_ptr<fpcfDistanceFilter<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new fpcfDistanceFilter and sets the 
             * fpcfPointCloud, a direction and a filter distance to filter the 
             * point cloud.
             *
             * @param[in] pointCloud sets the point cloud, which will be 
             *                       filtered
             * @param[in] direction sets the direction of the filtering
             * @param[in] filterDistance sets the maximum distance for points 
             *                           in the specified distance
             */
            fpcfDistanceFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, Direction direction, float filterDistance);

            /**
             * Constructor that creates a new fpcfDistanceFilter and sets the 
             * direction and a filter distance to filter a point cloud.
             *
             * @param[in] direction sets the direction of the filtering
             * @param[in] filterDistance sets the maximum distance for points 
             *                           in the specified distance
             */
            fpcfDistanceFilter(Direction direction, float filterDistance);

            /**
             * Destructor
             */
            virtual ~fpcfDistanceFilter();

            /**
             * Getter for the current direction of the distance filtering.
             * 
             * @return returns the current distance filter direction
             */
            Direction getFilterDirection() const;

            /**
             * Getter for the current filter distance
             *
             * @return returns the current filter distance
             */
            float getFilterDistance() const;

            /**
             * Sets a new filter direction
             * 
             * @param[in] direction the new filter direction
             */
            void setFilterDirection(Direction direction);

            /**
             * Sets a new filter distance
             * 
             * @param[in] filterDistance the new filter fistance
             */
            void setFilterDistance(float filterDistance);

            /**
             * Deep copy
             */
            fpcfDistanceFilter<PointT, DescriptorT>* clone() const;

            /**
             * Directly removes from the setted point clouds all points, which
             * are farer away than the specified distance in the specified 
             * direction from the coordinate system origin.
             */
            void filter();

        protected:

        private:
            typename pcl::PassThrough<PointT> passFilter_;
            Direction filterDirection_;
            float filterDistance_;

            fpcfDistanceFilter(); // hide default constructor

    };

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::fpcfDistanceFilter(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr pointCloud, Direction direction, float filterDistance)
        : fpcfFilter(pointCloud)
    {
        passFilter_.setInputCloud(pointCloud->getPointCloud());
        setFilterDirection(direction);
        setFilterDistance(filterDistance);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::fpcfDistanceFilter(Direction direction, float filterDistance)
        : fpcfFilter()
    {
        setFilterDirection(direction);
        setFilterDistance(filterDistance);
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::~fpcfDistanceFilter()
    {

    }

    template <typename PointT, typename DescriptorT>
    Direction
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::getFilterDirection() const
    {
        return filterDirection_;
    }

    template <typename PointT, typename DescriptorT>
    float
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::getFilterDistance() const
    {
        return filterDistance_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::setFilterDirection(Direction direction)
    {
        filterDirection_ = direction;
        if (direction == Direction::X)
        {
            passFilter_.setFilterFieldName("x");
        }
        else if (direction == Direction::Y)
        {
            passFilter_.setFilterFieldName("y");
        }
        else if (direction == Direction::Z)
        {
            passFilter_.setFilterFieldName("z");
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::setFilterDistance(float filterDistance)
    {
        filterDistance_ = filterDistance;
        passFilter_.setFilterLimits(0.0f, filterDistance);
    }

    template <typename PointT, typename DescriptorT>
    typename fpcfDistanceFilter<PointT, DescriptorT>*
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::clone() const
    {
        Direction filterDirection = getFilterDirection();
        float filterDistance = getFilterDistance();
        fpcf::fpcfDistanceFilter<PointT, DescriptorT> *fpcfDistanceFilter = new fpcf::fpcfDistanceFilter<PointT, DescriptorT>(filterDirection, filterDistance);
        return fpcfDistanceFilter;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfDistanceFilter<PointT, DescriptorT>::filter()
    {
        if (filterDistance_ > 0.0001)
        {
            passFilter_.setInputCloud(pointCloud_->getPointCloud());
            typename pcl::PointCloud<PointT>::Ptr filteredPointCloud(new pcl::PointCloud<PointT>());
            passFilter_.filter(*filteredPointCloud);
            pointCloud_->setPointCloud(filteredPointCloud);
            std::cout << "#Points after distance filter: " << filteredPointCloud->size() << std::endl;
        }
        else
        {
            std::cout << "distance filter too small, skipping distance filtering" << std::endl;
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFDISTANCEFILTER_H_