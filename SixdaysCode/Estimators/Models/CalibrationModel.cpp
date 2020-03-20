/*
 * XsensModel.cpp
 *
 *  Created on: 06.02.2019
 *      Author: tomlucas
 */

#include "CalibrationModel.hpp"

namespace zavi
::estimator::model {

	CalibrationModel::CalibrationModel(std::shared_ptr<plugin::SensorPlugin> sensor) :
	CALIBRATION_MODEL_TYPE(sensor) {
		plugin::IMU_Plugin * imu=static_cast<plugin::IMU_Plugin *>(sensor.get());
		Eigen::Matrix<double,6,6> stiffnes;
		stiffnes=stiffnes.Zero();
		stiffnes.block<3,3>(0,0)=imu->getAccelerationCov();
		stiffnes.block<3,3>(3,3)=imu->getAngularVelocityCov();
		input_measurement::stiffnes=stiffnes.diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
		orientation_measurement::stiffnes=orientation_measurement::stiffnes.Identity();
	}

	CalibrationModel::~CalibrationModel() {

	}

}
