/*
 * XsensModel.cpp
 *
 *  Created on: 06.02.2019
 *      Author: tomlucas
 */

#include "XsensModelWithBias.hpp"

namespace zavi
::estimator::model {


	XsensModelWithBias::XsensModelWithBias(std::shared_ptr<plugin::SensorPlugin> sensor) :
	XSENS_MODEL_WITH_BIAS_TYPE(sensor) {
		plugin::IMU_Plugin * imu=static_cast<plugin::IMU_Plugin *>(sensor.get());
		Eigen::Matrix<double,input_size,input_size> stiffnes=stiffnes.Zero();
		stiffnes.block<3,3>(0,0)=imu->getAccelerationCov()+Eigen::Matrix3d::Identity()*0.2;
		stiffnes.block<3,3>(3,3)=imu->getAngularVelocityCov();
		input_measurement::stiffnes=stiffnes.diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
		orientation_measurement::stiffnes=imu->getOrientationCov().diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
		//pos_velocity_measurement::stiffnes=pos_velocity_measurement::stiffnes.Identity();
	}

	XsensModelWithBias::~XsensModelWithBias() {

	}

}
