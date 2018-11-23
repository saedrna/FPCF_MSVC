/**
 * @file    fpcfPointCloudPreparation.h
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

#ifndef _FPCFPOINTCLOUDPREPARATION_H_
#define _FPCFPOINTCLOUDPREPARATION_H_

// STL
#include <time.h>

// Boost
#include <boost/thread.hpp>

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/filter/fpcfFilter.h>
#include <fpcf/registration/features/descriptors/fpcfDescriptor.h>
#include <fpcf/registration/features/detectors/fpcfDetector.h>
#include <fpcf/registration/features/fpcfPersistentFeatures.h>
#include <fpcf/registration/features/fpcfNormalEstimation.h>

namespace fpcf
{

    /**
     * This class prepares all added point cloud in parallel for the later 
     * point cloud registration steps. It filters the point clouds and computes
     * normals and point descriptors. Optionally it computes persistent 
     * features and detects keypoints of the point clouds. To compute the point
     * descriptors, a descriptor algorithm needs to be added manually. 
     */
    template <typename PointT, typename DescriptorT>
    class fpcfPointCloudPreparation
    {
        public:
            typedef boost::shared_ptr<fpcfPointCloudPreparation<PointT, DescriptorT>> Ptr;

            /**
             * Constructor that creates a new point cloud preparator for a 
             * specified point cloud.
             *
             * @param[in] fpcfPointCloud 
             */
            fpcfPointCloudPreparation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);

            /**
             * Constructor that creates a new empty point cloud preparator.
             */
            fpcfPointCloudPreparation();

            /**
             * Deep copy constructor
             */
            fpcfPointCloudPreparation(const fpcfPointCloudPreparation& fpcfPointCloudPreparator);

            /**
             * Destructor
             */
            virtual ~fpcfPointCloudPreparation();

            /**
             * Getter for the current vector of point clouds, which will be 
             * prepared.
             *
             * @return returns the vector of point clouds of the point cloud 
             *         preparator
             */
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> getPointClouds() const;

            /**
             * Getter for the current vecotr of filters, which will be applied 
             * to each point cloud during the preparation.
             *
             * @return returns the current vector of added point cloud filters
             */
            std::vector<typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr> getFilters() const;

            /**
             * Getter for the current descriptor algorithm, which is used to 
             * compute the point descriptors of the point clouds.
             *
             * @return return the current descriptor algorithm object
             */
            typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr getDescriptor() const;

            /**
             * Getter for the current keypoint detector algorithm, which is 
             * used to detect keypoints in the point clouds
             *
             * @return return the current keypoint detector algorithm object
             */
            typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr getDetector() const;

            /**
             * Getter for the current state of the persistent features 
             * computation. The persistent features of each point cloud are not
             * computed, if this state is 'false".
             *
             * @return returns the current flag state of the optional 
             *         persistent features computation
             */
            bool isCalculatingPersistantFeatures() const;

            /**
             * Getter for the current radius of the radius query during the 
             * estimation of the point normals.
             *
             * @return the current radius of the normal estimation
             */
            float getNormalEstimationRadius() const;

            /**
             * Adds a point cloud to the preparation process.
             *
             * @param[in] fpcfPointCloud the new point cloud, which will be 
             *                          prepared
             */
            void addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);

            /**
             * Sets the vector of point clouds for the preparation process.
             *
             * @param[in] fpcfPointClouds the vector of point clouds, which will
             *                           be prepared
             */
            void setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds);

            /**
             * Adds a new filter which will be applied to the point clouds. The
             * filters will be applied in the order they were added.
             *
             * @param[in] filter the new filter, which will be added
             */
            void addFilter(typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr filter);

            /**
             * Setter for a point descriptor algorithm, which will be used to 
             * compute the point descriptors of the point clouds. If no point 
             * descriptor algorithm is set, the point descriptors computation 
             * step is skipped.
             *
             * @param[in] fpcfDescriptor the point descriptor algorithm to 
             *                          describe the points of the point clouds
             */
            void setDescriptor(typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor);

            /**
             * Setter for a keypoint detector, which will be used to detect 
             * keypoints. If no keypoint detector is set, the keypoint 
             * detection step will be skipped.
             *
             * @param[in] fpcfDetector the keypoint detector algorithm to detect
             *                        a keypoint subset of the point clouds
             */
            void setDetector(typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr fpcfDetector);

            /**
             * Removes the current set keypoint detector. If no keypoint 
             * detector is set, the keypoint detection step will be skipped.
             */
            void removeDetector();

            /**
             * Sets the used radius to collect the point neighbors during the 
             * normal estimation.
             *
             * @param[in] normalEstimationRadius the radius of the radius query
             *                                   during the normal estimation
             */
            void setNormalEstimationRadius(float normalEstimationRadius);

            /**
             * Sets the flag for persistent features computation. If this flag 
             * is set to 'false', the computation of the persistent features 
             * will be skipped.
             *
             * @param[in] calculateState the flag, if the persistent features 
             *                           of the point clouds are computed
             */
            void calculatePersistentFeature(bool calculateState);

            /**
             * Invokes the parallelized preparation of all added point clouds. 
             * First the added filters are applied, then the normals of the 
             * remaining points are computed. Then optionally the persistent 
             * features are computed, if the according flag is set to 'true'. 
             * If a keypoint detector algorithm is set, the keypoints of the 
             * point clouds are computed. At the end the point descriptor 
             * algorithm is used to describe all remaining points or the 
             * keypoint subset respectively. All results can be accessed at the
             * fpcfPointCloudData input using the according methods.
             */
            void prepare();

        protected:

        private:
            std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds_;
            std::vector<typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr> fpcfFilters_;
            typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor_;
            typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr fpcfDetector_;
            bool calculatePersistentFeatures_;
            float normalSearchRadius_;

            void creatPreparationCopy(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void preparePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void applyFilters(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void calculateNormals(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void calculateKeypoints(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void calculateDescriptors(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
            void calculatePersistentFeatures(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud);
    };


    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::fpcfPointCloudPreparation()
    {
        calculatePersistentFeatures_ = false;
        normalSearchRadius_ = 0.03f;
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::fpcfPointCloudPreparation(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfPointCloud_ = fpcfPointCloud;
        calculatePersistentFeatures_ = false;
    }

    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::fpcfPointCloudPreparation(const fpcfPointCloudPreparation& fpcfPointCloudPreparator)
    {
        std::vector<fpcf::fpcfFilter<PointT, DescriptorT>::Ptr> fpcfFilters = fpcfPointCloudPreparator.getFilters();
        for (int filNr = 0; filNr < fpcfFilters.size(); filNr++)
        {
            typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr fpcfFilter(fpcfFilters.at(filNr)->clone());
            addFilter(fpcfFilter);
        }
        if (fpcfPointCloudPreparator.getDescriptor())
        {
            typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor(fpcfPointCloudPreparator.getDescriptor()->clone());
            setDescriptor(fpcfDescriptor);
        }
        if (fpcfPointCloudPreparator.getDetector())
        {
            typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr fpcfDetector(fpcfPointCloudPreparator.getDetector()->clone());
            setDetector(fpcfDetector);
        }
        calculatePersistentFeature(fpcfPointCloudPreparator.isCalculatingPersistantFeatures());
        setNormalEstimationRadius(fpcfPointCloudPreparator.getNormalEstimationRadius());
    }
      
    template <typename PointT, typename DescriptorT>
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::~fpcfPointCloudPreparation()
    {
        
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr>
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::getPointClouds() const
    {
        return pointClouds_;
    }

    template <typename PointT, typename DescriptorT>
    std::vector<typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr> 
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::getFilters() const
    {
        return fpcfFilters_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::getDescriptor() const
    {
        return fpcfDescriptor_;
    }

    template <typename PointT, typename DescriptorT>
    typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::getDetector() const
    {
        return fpcfDetector_;
    }

    template <typename PointT, typename DescriptorT>
    bool
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::isCalculatingPersistantFeatures() const
    {
        return calculatePersistentFeatures_;
    }

    template <typename PointT, typename DescriptorT>
    float
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::getNormalEstimationRadius() const
    {
        return normalSearchRadius_;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::addPointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfPointClouds_.push_back(fpcfPointCloud);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::setPointClouds(std::vector<typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr> fpcfPointClouds)
    {
        fpcfPointClouds_ = fpcfPointClouds;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::addFilter(typename fpcf::fpcfFilter<PointT, DescriptorT>::Ptr filter)
    {
        fpcfFilters_.push_back(filter);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::setDescriptor(typename fpcf::fpcfDescriptor<PointT, DescriptorT>::Ptr fpcfDescriptor)
    {
        fpcfDescriptor_ = fpcfDescriptor;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::setDetector(typename fpcf::fpcfDetector<PointT, DescriptorT>::Ptr fpcfDetector)
    {
        fpcfDetector_ = fpcfDetector;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::removeDetector()
    {
        fpcfDetector_.reset();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::calculatePersistentFeature(bool calculateState)
    {
        calculatePersistentFeatures_ = calculateState;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::setNormalEstimationRadius(float normalEstimationRadius)
    {
        normalSearchRadius_ = normalEstimationRadius;
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::creatPreparationCopy(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        pcl::PointCloud<PointT>::Ptr copyPointCloud(new pcl::PointCloud<PointT>(*fpcfPointCloud->getPointCloud()));
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr copyPcfPointCloud(new fpcf::fpcfPointCloudData<PointT, DescriptorT>(copyPointCloud, fpcfPointCloud->getId() + "copy"));
        fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr mostDownsampledPointCloud = fpcfPointCloud->getMostDownsampledPointCloud();
        mostDownsampledPointCloud->setDownsampledPointCloud(copyPcfPointCloud);
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::applyFilters(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        for(int i = 0; i < fpcfFilters_.size(); i++)
        {
            fpcf::fpcfFilter<PointT, DescriptorT>::Ptr currentFilter = fpcfFilters_.at(i);
            currentFilter->setPointCloud(fpcfPointCloud->getMostDownsampledPointCloud());
            currentFilter->filter();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::calculateNormals(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcf::fpcfNormalEstimation<PointT, DescriptorT> fpcfNormalEstimation(fpcfPointCloud->getMostDownsampledPointCloud(), normalSearchRadius_);
        fpcfNormalEstimation.estimatePointNormals();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::calculateKeypoints(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfDetector_->setPointCloud(fpcfPointCloud->getMostDownsampledPointCloud());
        fpcfDetector_->detectKeypoints();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::calculateDescriptors(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcfDescriptor_->setPointCloud(fpcfPointCloud->getMostDownsampledPointCloud());
        // if existing, calculate only descriptors of keypoints instead of all points
        if (fpcfDetector_)
        {
            fpcfDescriptor_->calculateKeypointDescriptors();
        }
        else
        {
            fpcfDescriptor_->calculatePointDescriptors();
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::calculatePersistentFeatures(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        fpcf::fpcfPersistentFeatures<PointT, DescriptorT> fpcfPersistentFeatures(fpcfPointCloud->getMostDownsampledPointCloud());
        fpcfPersistentFeatures.calculatePersistentFeatures();
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::prepare()
    {
        boost::thread_group thrdGrp;
        std::vector<fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>*> preparators;
        // create threads, if more than one point cloud exists
        for (int pcNr = 1; pcNr < fpcfPointClouds_.size(); pcNr++)
        {
            fpcf::fpcfPointCloudPreparation<PointT, DescriptorT> *fpcfPointCloudPreparator = new fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>(*this);
            fpcfPointCloudPreparator->addPointCloud(fpcfPointClouds_.at(pcNr));
            preparators.push_back(fpcfPointCloudPreparator);
            thrdGrp.create_thread(boost::bind(&fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::prepare, fpcfPointCloudPreparator));
        }
        if(fpcfPointClouds_.size() > 0)
        {
            preparePointCloud(fpcfPointClouds_.at(0));
        }
        thrdGrp.join_all();
        for (int prepNr = 0; prepNr < preparators.size(); prepNr++)
        {
            delete preparators.at(prepNr);
        }
    }

    template <typename PointT, typename DescriptorT>
    void
    fpcf::fpcfPointCloudPreparation<PointT, DescriptorT>::preparePointCloud(typename fpcf::fpcfPointCloudData<PointT, DescriptorT>::Ptr fpcfPointCloud)
    {
        if (!fpcfPointCloud->isPrepared())
        {
            // create copy for preparation
            creatPreparationCopy(fpcfPointCloud);

            // filter
            applyFilters(fpcfPointCloud);

            // normals
            calculateNormals(fpcfPointCloud);

            // if activated, calculate persistent features
            if(calculatePersistentFeatures_)
            {
                calculatePersistentFeatures(fpcfPointCloud);
            }

            // keypoints calculation, if detector is set
            if (fpcfDetector_)
            {
                calculateKeypoints(fpcfPointCloud);
            }

            // calculate descriptors
            if (fpcfDescriptor_)
            {
                calculateDescriptors(fpcfPointCloud);
            }
            fpcfPointCloud->isPrepared(true);
        }
    }

} // end namespace fpcf

#endif // #ifndef _FPCFPOINTCLOUDPREPARATION_H_