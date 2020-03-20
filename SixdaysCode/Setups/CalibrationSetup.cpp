/*
 * ODE_setupts.cpp
 *
 *  Created on: 06.02.2019
 *      Author: tomlucas
 */

#include "SetupBase.hpp"
#include "../Estimators/PseudoMeasurements.hpp"
#include "../Estimators/CalibrationBatch.hpp"
#include "../Plugins/FileIMUPlugin.hpp"
#include <CameraToVideo.hpp>
namespace zavi
::setups {


	void CalibrationSetup() {
		//Measurement generator
		std::shared_ptr<plugin::FileIMUPlugin> imu(new plugin::FileIMUPlugin(cfg->lookup("calibration_file"),0,cfg->lookup("calibration_end"),true));
		//estimator
		std::shared_ptr<estimator::CalibrationBatchEstimator> estimator=estimator::CalibrationBatchEstimator::createCalibrationBatchEstimator(imu,true);
		imu->addBatchCallback(estimator->batchCallback,estimator);
		estimator->addConstraint(std::make_shared<estimator::pseudo_measurement::MeasurementConstraint< estimator::pseudo_measurement::OnlyRollConstrain,estimator::CalibrationBatchEstimator> >(), NULL);

		double freq=100.;
		ThreadedObject::start_thread(imu,freq,RUNNING);
		}

}

