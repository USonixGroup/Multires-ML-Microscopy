#ifndef _OPENCV_MWFLANN_PRECOMP_HPP_
#define _OPENCV_MWFLANN_PRECOMP_HPP_

#include <cstdio>
#include <cstdarg>
#include <sstream>

#ifdef COMPILE_FOR_VISION_BUILTINS
/* codegen does not need the following header */
#include "cvconfig.h"
#endif
#include "opencv2/core.hpp"

#include "opencv2/flann/miniflann.hpp"
#include "opencv2/flann/dist.h"
#include "opencv2/flann/index_testing.h"
#include "opencv2/flann/params.h"
#include "opencv2/flann/saving.h"
#include "opencv2/flann/general.h"
#include "opencv2/flann/dummy.h"

// index types
#include "opencv2/flann/all_indices.h"
#include "opencv2/flann/flann_base.hpp"

#endif
