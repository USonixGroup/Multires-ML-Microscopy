////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAM.cpp
//
// MCOS C++ API interface for MonoVisualSLAM
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#include <MonoVisualSLAM.hpp>

namespace vision {
    void MonoVisualSLAM::initializeClass(mcos::COSClass* cosClass,
        const mcos::COSClient* privateClient) {

        BaseVisualSLAM<vision::vslam::MonoVisualSLAMProxy>::initializeClass(cosClass, privateClient);

        cosClass->addMethod("storeGravityRotationAndScale", mcos::PublicMember, &vision::vslam::MonoVisualSLAMProxy::storeGravityRotationAndScale);

    }
} // namespace vision