#ifndef _VISION_G2O_BUNDLEADJUST_TYPES_HPP_
#define _VISION_G2O_BUNDLEADJUST_TYPES_HPP_

#include "g2ocv/libmwg2ocv_util.hpp"

#include "mex.h"
#include "matrix.h"
#include <matrix/smart_ptr.hpp>

#include <Eigen/StdVector>
#include "g2o/types/sba/types_six_dof_expmap.h"

namespace vision {

	namespace bundleAdjust {

		class LIBMWG2OCV_API VertexCamera : public g2o::BaseVertex<6, g2o::SE3Quat> {
		public:
			// Camera parameters
			double* focal_length = nullptr;
			double* principle_point = nullptr;
			double skew;
			size_t numRadialCoefs;
			double* kr = nullptr;
			double* kt = nullptr;

			VertexCamera();
			VertexCamera(double* focal,
				double* center,
				double sk,
				size_t nRC,
				double* kr_tmp,
				double* kt_tmp);

			~VertexCamera();

			virtual bool read(std::istream&) override;
			virtual bool write(std::ostream&) const override;

			void setToOriginImpl() override;
			void oplusImpl(const double*) override;

			// Project world points to the image
			inline Eigen::Vector2d cam_map(const Eigen::Vector3d& Xc) const {

				Eigen::Vector2d proj;
				if (kr == nullptr) {
					// Without distortion
					proj(0) = Xc(0) / Xc(2);
					proj(1) = Xc(1) / Xc(2);
				}
				else {
					// With distortion
					Eigen::Vector2d P;
					P(0) = Xc(0) / Xc(2);
					P(1) = Xc(1) / Xc(2);

					double r2 = P.dot(P);

					if (numRadialCoefs == 3) {
						proj = P * (1 + kr[0] * r2 + kr[1] * r2 * r2 + kr[2] * r2 * r2 * r2);
					}
					else {
						proj = P * (1 + kr[0] * r2 + kr[1] * r2 * r2);
					}

					proj(0) += 2 * kt[0] * P(0) * P(1) + kt[1] * (r2 + 2 * P(0) * P(0));
					proj(1) += kt[0] * (r2 + 2 * P(1) * P(1)) + 2 * kt[1] * P(0) * P(1);
				}

				Eigen::Vector2d res;

				res(0) = focal_length[0] * (proj(0) + skew * proj(1)) + principle_point[0];
				res(1) = focal_length[1] * proj(1) + principle_point[1];

				return res;
			}
		};

		class LIBMWG2OCV_API EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, vision::bundleAdjust::VertexCamera> {
		public:
			double* Xp;

			EdgeProjection();
			~EdgeProjection();

			virtual bool read(std::istream&) override;
			virtual bool write(std::ostream&) const override;

			// Compute reprojection error
			Eigen::Vector2d returnError();
			void computeError() override;

			// Compute Jacobian
			using g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, vision::bundleAdjust::VertexCamera>::linearizeOplus;
			void linearizeOplus() override;

		};

	}
}
#endif /*_VISION_G2O_BUNDLEADJUST_TYPES_HPP_*/