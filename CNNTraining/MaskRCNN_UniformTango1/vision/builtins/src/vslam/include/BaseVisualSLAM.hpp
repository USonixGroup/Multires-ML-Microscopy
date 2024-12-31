////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAM.hpp
//
//  This header implements the MCOS C++ API for the BaseVisualSLAM interface
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BASEVISUALSLAM_HPP
#define BASEVISUALSLAM_HPP

#include "mcos_factory/mi.hpp"

namespace vision {
    template<typename TProxy>
    class BaseVisualSLAM : public mcos::factory::HandleClassInfo<TProxy, true> // <class, deleteOnDeallocate>
    {
      public:
        BaseVisualSLAM(const std::string& name, const std::string& description)
            : mcos::factory::HandleClassInfo<TProxy, true>(name, description)
        {}

        void initializeClass(mcos::COSClass* cosClass,
            const mcos::COSClient* privateClient) override {

            mcos::factory::McosFactory f(cosClass, privateClient);

            f.Constructor<TProxy>();

            cosClass->addMethod("configure", mcos::PublicMember, &TProxy::configure);

            cosClass->addMethod("addFrame", mcos::PublicMember, &TProxy::addFrame);

            cosClass->addMethod("getCameraPoses", mcos::PublicMember, &TProxy::getCameraPoses);

            cosClass->addMethod("getKeyFrameIndex", mcos::PublicMember, &TProxy::getKeyFrameIndex);

            cosClass->addMethod("getViewIDs", mcos::PublicMember, &TProxy::getViewIDs);

            cosClass->addMethod("getWorldPoints", mcos::PublicMember, &TProxy::getWorldPoints);

            cosClass->addMethod("getNumTrackedPoints", mcos::PublicMember, &TProxy::getNumTrackedPoints);

            cosClass->addMethod("hasNewKeyFrame", mcos::PublicMember, &TProxy::hasNewKeyFrame);

            cosClass->addMethod("isDone", mcos::PublicMember, &TProxy::isDone);

            cosClass->addMethod("isInitialized", mcos::PublicMember, &TProxy::isMapInitialized);

            cosClass->addMethod("isLoopRecentlyClosed", mcos::PublicMember, &TProxy::isLoopRecentlyClosed);

            cosClass->addMethod("getLogFileName", mcos::PublicMember, &TProxy::getLogFileName);

            cosClass->addMethod("reset", mcos::PublicMember, &TProxy::reset);

            cosClass->addMethod("getViewIMUMeasurements", mcos::PublicMember, &TProxy::getViewIMUMeasurements);
        }
    };
} // namespace vision

#endif // BASEVISUALSLAM_HPP