/*
 * @file
 * NOIntegratorUKF.hpp
 *
 *  Created on: 03.09.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_LAMBDAPATHUKFEXTENDED_HPP
#define ESTIMATORS_LAMBDAPATHUKFEXTENDED_HPP

#include "UKF.hpp"
#include "Models/LambdaPathWithVelocityModel.hpp"

#include <Eigen_Utils.hpp>

#include "../Plugins/imu_plugin.hpp"
#include "StateBoxes/OrientationBox.hpp"

#include <iostream>

namespace zavi
::estimator {
	/**
	 * makes use of LambdaPathWithVelocityModel and classic UKF
	 * Improved estimation of the position by estimating the velocity via forward velocity prior
	 */
	class LambdaPathUKFExtended: public UKF<zavi::estimator::model::LambdaPathWithVelocityModel> {

	public :
		enum CallbackIndices {CI_AFTER_IMU_CALLBACK,     //!< at end of imu callback
			CI_AT_START_IMU_CALLBACK};					//!< at start of imu callback
	protected:
		zavi::eigen_util::FuncCaller lambda_path;
		/**
		 * Protected constructor only call from createUKFIntegrator
		 * @param imu pointer to main imu
		 */

		//template<typename functor>
		LambdaPathUKFExtended(std::shared_ptr<plugin::IMU_Plugin> imu, eigen_util::FuncCaller & func):UKF<zavi::estimator::model::LambdaPathWithVelocityModel>(imu), lambda_path(func) {

			state_boxes::OrientationSpeedBox::INNER_T<double> orient_std=orient_std.Ones()*0.;

			orient_std(3)=orient_std(4)=orient_std(5)=1.;
			//orient_std(0,0)=orient_std(1,0)=orient_std(2,0)=0.1;
			std::get<state_boxes::OrientationSpeedBox>(box_model.boxes).std=orient_std;
			state_boxes::LambdaPathBoxNoAcceleration::INNER_T<double > std=std.Zero();
			std(0)=10.;
			std(1)=0.1;
			std::get<state_boxes::LambdaPathBoxNoAcceleration>(box_model.boxes).std=std;
			state_boxes::CartesianBox::INNER_T<double > cart_std=cart_std.Zero();
			cart_std(6)=cart_std(7)=cart_std(8)= 100;
			std::get<state_boxes::CartesianBox>(box_model.boxes).std=cart_std;
		}

	public:
		zavi::eigen_util::FuncCaller getFuncCaller() {
			return lambda_path;
		}
		/**
		 * Callback for imu
		 * @param plug the imu plugin which has new data
		 * @param estimator the estimator which gets the new data
		 * @param time the current time stamp
		 */
		static void imuCallback(plugin::SensorPlugin * plug,
				void* estimator, double time) {
			static double last_time=time-0.0001;     // fix for zero start time
			double time_diff=time-last_time;
			LambdaPathUKFExtended * esti = static_cast<LambdaPathUKFExtended *>(estimator);
			esti->saveStatesForSmoothing(time_diff);
			esti->notifyCallbacks(CI_AT_START_IMU_CALLBACK);
			plugin::IMU_Plugin *imu = static_cast<plugin::IMU_Plugin*>(plug);
			/*	esti->state_vector(2,0)=(*imu->getAcceleration())(0,0);
			 esti->covariance(2,2)=(*imu->getAccelerationCov())(0,0);
			 esti->state_vector.block<3,1>(esti->box_model.orientation_speed_start,0)=*(imu->getAngularVelocity());

			 esti->covariance.block<3,3>(esti->box_model.orientation_start+3,esti->box_model.orientation_start+3)=*(imu->getAngularVelocityCov());*/
			//		zavi::plot::liveHeatMap<0>(esti->covariance,"Covariance of UKF");
			//zavi::plot::liveStateDraw<1>(*imu->getAngularVelocity(), "AR");
			//zavi::plot::liveStateDraw<2>(*imu->getAcceleration(), "Acceleration");
			//	zavi::plot::liveStateDraw<3>(esti->getBoxModel().acceleration_measurement(esti->state_vector,NULL), "estimated acceleration");
			//esti->state_vector.block<3,1>(6,0)=(imu->acceleration_inc);
			Eigen::Matrix<double,1,1> target=target.Zero();
			Eigen::Matrix<double,1,1> lambda_noise=lambda_noise.Identity()*0.2;
			esti->measurementStep(target,time_diff,esti->box_model.dot_lambda_measurement<double>,lambda_noise);
			//esti->state_vector(esti->box_model.lambda_velocity_start,0)=esti->box_model.getVelocity(esti->getStateVector()).norm();
			esti->dynamicStep(time_diff);
			//zavi::plot::liveStateDraw<1,double,9>(esti->state_vector.block<9,1>(0,0),"Cart Vector");
			//zavi::plot::liveStateDraw<2,double,12>(esti->state_vector.block<12,1>(11,0),"Rotation Vector");
			Eigen::Matrix3d noise=imu->getAccelerationCov()+Eigen::Matrix3d::Identity()*0.2;
			Eigen::Matrix3d orient_noise=imu->getAngularVelocityCov();
			esti->measurementStep(imu->getAngularVelocity(),time_diff,esti->box_model.orientation_change_measurement<double>,orient_noise);
			esti->measurementStep(imu->getAcceleration(),time_diff,esti->box_model.acceleration_measurement<double>,noise);

			last_time=time;
			esti->notifyCallbacks(CI_AFTER_IMU_CALLBACK);
		}

		/**
		 * Create an IntegratorEstimatorUKF shared pointer
		 * @param imu the imu to read the basic data
		 * @return shared pointer to the estimator
		 */
		//template<typename functor>
		static std::shared_ptr<zavi::estimator::LambdaPathUKFExtended> createLambdaPathUKFExtended(std::shared_ptr<plugin::IMU_Plugin> imu, eigen_util::FuncCaller func) {
			std::shared_ptr<zavi::estimator::LambdaPathUKFExtended > estimator(new LambdaPathUKFExtended(imu,func));
			imu->addDataCallback(imuCallback,estimator);
			return estimator;
		}

		virtual Eigen::Vector3d getEstimatedPosition() {
			return lambda_path(state_vector(9,0));
		}

		virtual Eigen::Quaterniond getEstimatedOrientation() {
			return Eigen::Quaterniond(Eigen::Matrix3d(state_vector.block<9,1>(MODEL_TYPE::orientation_start,0).data()));     //< static orientation
		}

		virtual Eigen::Matrix3d getEstimatedPositionError() {
			double sigma= sqrt(covariance(9,9));
			double l=state_vector(9,0);
			Eigen::Vector3d mu=lambda_path(l);
			Eigen::Matrix3d variance=variance.Zero();
			for(int i=-1; i<=1; i++) {
				Eigen::Vector3d diff=lambda_path(i*sigma+l)-mu;
				variance+=diff*diff.transpose();
			}
			variance/=2.;
			return variance;
		}

		virtual void run(double freq, bool (*running)()) {

		}

	private:
		//Eigen::Matrix<double, NOUKF_STATE_DIM, NOUKF_STATE_DIM> state_transition;     //< state transition matrix
		//Eigen::Matrix<double, NOUKF_STATE_DIM, NOUKF_INPUT_DIM> input_gain;//< input matrix
	};

}
//zavi::estimator

#endif /* ESTIMATORS_LAMBDAPATHUKFEXTENDED_HPP */
