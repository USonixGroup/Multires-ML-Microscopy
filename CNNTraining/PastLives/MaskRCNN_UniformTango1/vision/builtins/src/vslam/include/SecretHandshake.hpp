/*
 * SecretHandshake.hpp
 *
 * Implements functionality for license checkout for Navigation Toolbox
 *
 * Copyright 2023 The MathWorks, Inc.
 */

#ifndef SECRETHANDSHAKE_HPP
#define SECRETHANDSHAKE_HPP

#include "package.h"
#include "services.h"
#include "services/lmgr/lmgrcore.hpp" // for svLmCheckoutFeature

#include "fl/except/MsgIDException.hpp"
#include "resources/vision/visionlib.hpp"

namespace vision{
    namespace vslam {

        static inline void checkoutNavLicense() {

            // Checkout Navigation Toolbox license
            int navLicenseStatusCode = svLmCheckoutFeature("navigation_toolbox", false, nullptr);
            bool hasNavLicense = (navLicenseStatusCode == 0);

            // Throw an error if checkout failed
            if (!hasNavLicense) {
                throw fl::except::MakeException(vision::visionlib::noNavLicense());
            }
        }
    } // end namespace vslam
}
#endif