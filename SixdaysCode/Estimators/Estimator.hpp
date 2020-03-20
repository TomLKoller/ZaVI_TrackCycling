/*
 * @file Holds the base class for estimators
 * Estimator.hpp
 *
 *  Created on: 24.07.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_ESTIMATOR_HPP_
#define ESTIMATORS_ESTIMATOR_HPP_

#include <Eigen/Core>

#include <Eigen/Geometry>
#include <threaded_object.hpp>
#include <Callbacker.hpp>
namespace zavi {
namespace estimator {


/**
 * Base class for all estimators
 */
class Estimator:  public Callbacker ,public ThreadedObject{
public:

	virtual ~Estimator() {
		DLOG(INFO) << "Estimator got destroyed";
	}

	/**
	 * returns the current estimated state
	 *@return the current estimated position
	 */
	virtual Eigen::Vector3d getEstimatedPosition()=0;

	/**
	 * Gives the estimated Orientation
	 * @return the estimated orientation as quaternion
	 */
	virtual Eigen::Quaterniond getEstimatedOrientation()=0;

	/**
	 * Gives the covariance of the position
	 *
	 * @return the estimated covariance of the estimated position
	 *
	 */
	virtual Eigen::Matrix<double, 3, 3> getEstimatedPositionError()=0;

private:
};

} /* namespace estimator */
} /* namespace zavi */

#endif /* ESTIMATORS_ESTIMATOR_HPP_ */
