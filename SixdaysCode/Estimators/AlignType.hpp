/*
 * AlignType.hpp
 *
 *  Created on: 21.02.2019
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_ALIGNTYPE_HPP_
#define ESTIMATORS_ALIGNTYPE_HPP_

#include <Eigen/Core>

namespace zavi::estimator{
	/**
	 * Defines which type to use for the calibration matrix between imu and body frame
	 */
	template<typename T>
	using ALIGNMENT_TYPE=Eigen::Matrix<T,4,4>;
	static constexpr int ALIGNMENT_SIZE=16;
}

#endif /* ESTIMATORS_ALIGNTYPE_HPP_ */
