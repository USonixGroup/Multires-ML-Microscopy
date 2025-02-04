////////////////////////////////////////////////////////////////////////////////////////////
//////////////  Defines datatypes used in bundle adjustment  ///////////////////////////////

// Copyright 2020-2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////////////////

#include "g2ocv/libmwg2ocv_util.hpp"
#include "g2ocv/visionG2OBundleAdjustTypes.hpp"

using namespace vision::bundleAdjust;

VertexCamera::VertexCamera() {};

VertexCamera::VertexCamera(double* focal,
	double* center,
	double sk,
	size_t nRC,
	double* kr_tmp,
	double* kt_tmp)
	: focal_length(focal)
	, principle_point(center)
	, skew(sk)
	, numRadialCoefs(nRC)
	, kr(kr_tmp)
	, kt(kt_tmp) {
} // Initializer list

VertexCamera::~VertexCamera() {}

// I/O, required methods
bool VertexCamera::read(std::istream& /*is*/) {
	return false;
}
bool VertexCamera::write(std::ostream& /*os*/) const {
	return false;
}

void VertexCamera::setToOriginImpl() {
	_estimate = g2o::SE3Quat();
}

void VertexCamera::oplusImpl(const double* update_) {
	Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
	setEstimate(g2o::SE3Quat::exp(update) * estimate());
}


EdgeProjection::EdgeProjection() {}

EdgeProjection::~EdgeProjection() {}

// I/O, required methods
bool EdgeProjection::read(std::istream&) {
	return false;
}
bool EdgeProjection::write(std::ostream&) const {
	return false;
}

// Compute reprojection error
Eigen::Vector2d EdgeProjection::returnError() {
	auto vj = static_cast<VertexCamera*>(_vertices[1]);
	auto vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
	Eigen::Vector2d obs(_measurement);
	Eigen::Vector2d reprojection = vj->cam_map(vj->estimate().map(vi->estimate()));
	return obs - reprojection;
}

void EdgeProjection::computeError() {
	_error = returnError();
}

// Compute Jacobian
void EdgeProjection::linearizeOplus() {
	auto vj = static_cast<VertexCamera*>(_vertices[1]);
	g2o::SE3Quat T(vj->estimate());
	auto vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
	Eigen::Vector3d xyz = vi->estimate();
	Eigen::Vector3d xyz_trans = T.map(xyz);

	double x = xyz_trans[0];
	double y = xyz_trans[1];
	double z = xyz_trans[2];
	double z_2 = z * z;

	Eigen::Matrix<double, 2, 3, Eigen::ColMajor> tmp;
	tmp(0, 0) = vj->focal_length[0];
	tmp(0, 1) = 0;
	tmp(0, 2) = -x / z * vj->focal_length[0];

	tmp(1, 0) = 0;
	tmp(1, 1) = vj->focal_length[1];
	tmp(1, 2) = -y / z * vj->focal_length[1];

	_jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();

	_jacobianOplusXj(0, 0) = x * y / z_2 * vj->focal_length[0];
	_jacobianOplusXj(0, 1) = -(1 + (x * x / z_2)) * vj->focal_length[0];
	_jacobianOplusXj(0, 2) = y / z * vj->focal_length[1];
	_jacobianOplusXj(0, 3) = -1. / z * vj->focal_length[1];
	_jacobianOplusXj(0, 4) = 0;
	_jacobianOplusXj(0, 5) = x / z_2 * vj->focal_length[0];

	_jacobianOplusXj(1, 0) = (1 + y * y / z_2) * vj->focal_length[1];
	_jacobianOplusXj(1, 1) = -x * y / z_2 * vj->focal_length[1];
	_jacobianOplusXj(1, 2) = -x / z * vj->focal_length[0];
	_jacobianOplusXj(1, 3) = 0;
	_jacobianOplusXj(1, 4) = -1. / z * vj->focal_length[0];
	_jacobianOplusXj(1, 5) = y / z_2 * vj->focal_length[1];
}